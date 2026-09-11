//! Exact-attempt provider-profile binding for Mycelix integration execution.
//!
//! This crate is intentionally **not** an execution-authority source and does
//! not expose a provider client, payload-materialization API, or dispatch call.
//! It closes one narrower theorem required by INT-04:
//!
//! ```text
//! exact INT-03 ExecutionClaim
//! + exact typed IntegrationCommand
//! + current signed provider execution profile
//!     -> QualifiedExecutionBinding
//! ```
//!
//! The positive binding proves semantic identity compatibility only. A later
//! authority composer must additionally supply fresh, non-forgeable execution
//! authority before provider payload materialization or `DispatchStarted`.

use ed25519_dalek::{Signature, Verifier, VerifyingKey};
use mycelix_integration_core::{
    CanonicalEncodeV1, ConnectorInstanceId, ContentCommitment, DigestAlgorithm,
    ExternalObjectType, ExternalOperationKind, ExternalSystemId, IdempotencyKey,
    IntegrationCommand, IntegrationCommandId, SemanticProfileId, SideEffectClass,
};
use mycelix_integration_runtime::ExecutionClaim;
use thiserror::Error;

const PROFILE_PROTOCOL_V1: &str = "mycelix-integration-provider-execution-profile-v1";
const DOMAIN_PROFILE: &[u8] = b"mycelix/integration/provider-execution-profile/v1";
const DOMAIN_TRUST_ROOT: &[u8] = b"mycelix/integration/provider-profile-trust-root/v1";
const DOMAIN_BINDING: &[u8] = b"mycelix/integration/execution-binding/v1";
const MAX_KEY_ID_BYTES: usize = 256;
const MAX_PROFILE_PAYLOAD_BYTES: u32 = 64 * 1024 * 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProviderIdempotencySemantics {
    Unsupported,
    /// The provider contract promises request-key deduplication for at least
    /// `retention_ms`; lookup support means reconciliation can query by that key.
    RequestKey {
        retention_ms: u64,
        lookup_supported: bool,
        duplicate_returns_original: bool,
    },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProviderReconciliationMode {
    None,
    ExactOperation,
    IdempotencyKey,
    ObjectLookup,
    CursorScan,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProviderExecutionProfile {
    pub protocol_version: String,
    pub profile_id: SemanticProfileId,
    pub generation: u64,
    pub signer_key_id: String,
    pub connector_instance: ConnectorInstanceId,
    pub system: ExternalSystemId,
    pub operation_kind: ExternalOperationKind,
    pub required_target_type: Option<ExternalObjectType>,
    pub side_effect_class: SideEffectClass,
    pub idempotency: ProviderIdempotencySemantics,
    pub reconciliation: ProviderReconciliationMode,
    pub materializer_release: ContentCommitment,
    pub max_payload_bytes: u32,
    pub not_before_ms: i64,
    pub not_after_ms: i64,
}

impl ProviderExecutionProfile {
    pub fn validate(&self) -> Result<(), QualificationError> {
        if self.protocol_version != PROFILE_PROTOCOL_V1 {
            return Err(QualificationError::WrongProtocol);
        }
        validate_key_id(&self.signer_key_id)?;
        if self.generation == 0 {
            return Err(QualificationError::InvalidGeneration);
        }
        if self.not_before_ms < 0
            || self.not_after_ms < 0
            || self.not_before_ms > self.not_after_ms
        {
            return Err(QualificationError::InvalidValidityWindow);
        }
        if self.max_payload_bytes == 0 || self.max_payload_bytes > MAX_PROFILE_PAYLOAD_BYTES {
            return Err(QualificationError::InvalidPayloadLimit);
        }

        if let ProviderIdempotencySemantics::RequestKey { retention_ms, .. } = self.idempotency
            && retention_ms == 0
        {
            return Err(QualificationError::InvalidIdempotencyContract);
        }

        if self.reconciliation == ProviderReconciliationMode::IdempotencyKey {
            match self.idempotency {
                ProviderIdempotencySemantics::RequestKey {
                    retention_ms,
                    lookup_supported: true,
                    ..
                } if retention_ms > 0 => {}
                _ => return Err(QualificationError::InvalidIdempotencyContract),
            }
        }
        Ok(())
    }

    pub fn canonical_preimage_v1(&self) -> Result<Vec<u8>, QualificationError> {
        self.validate()?;
        let mut out = DOMAIN_PROFILE.to_vec();
        push_str(&mut out, &self.protocol_version);
        push_str(&mut out, self.profile_id.as_str());
        out.extend_from_slice(&self.generation.to_be_bytes());
        push_str(&mut out, &self.signer_key_id);
        push_str(&mut out, self.connector_instance.as_str());
        push_str(&mut out, self.system.as_str());
        push_str(&mut out, self.operation_kind.as_str());
        match &self.required_target_type {
            Some(value) => {
                out.push(1);
                push_str(&mut out, value.as_str());
            }
            None => out.push(0),
        }
        out.push(side_effect_tag(self.side_effect_class));
        match self.idempotency {
            ProviderIdempotencySemantics::Unsupported => out.push(0),
            ProviderIdempotencySemantics::RequestKey {
                retention_ms,
                lookup_supported,
                duplicate_returns_original,
            } => {
                out.push(1);
                out.extend_from_slice(&retention_ms.to_be_bytes());
                out.push(u8::from(lookup_supported));
                out.push(u8::from(duplicate_returns_original));
            }
        }
        out.push(reconciliation_tag(self.reconciliation));
        push_commitment(&mut out, &self.materializer_release);
        out.extend_from_slice(&self.max_payload_bytes.to_be_bytes());
        out.extend_from_slice(&self.not_before_ms.to_be_bytes());
        out.extend_from_slice(&self.not_after_ms.to_be_bytes());
        Ok(out)
    }

    pub fn profile_commitment(&self) -> Result<ContentCommitment, QualificationError> {
        Ok(ContentCommitment::sha256(&self.canonical_preimage_v1()?))
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SignedProviderExecutionProfile {
    pub profile: ProviderExecutionProfile,
    pub signature: [u8; 64],
}

/// Current trust-root input supplied by the enclosing authority/configuration
/// system. Qualification is explicitly relative to this root; constructing a
/// root does not itself grant execution authority.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProviderProfileTrustRoot {
    key_id: String,
    verifying_key: [u8; 32],
    current_generation: u64,
    root_commitment: ContentCommitment,
}

impl ProviderProfileTrustRoot {
    pub fn new(
        key_id: impl Into<String>,
        verifying_key: [u8; 32],
        current_generation: u64,
    ) -> Result<Self, QualificationError> {
        let key_id = key_id.into();
        validate_key_id(&key_id)?;
        if current_generation == 0 {
            return Err(QualificationError::InvalidGeneration);
        }
        VerifyingKey::from_bytes(&verifying_key)
            .map_err(|_| QualificationError::InvalidVerifyingKey)?;

        let mut preimage = DOMAIN_TRUST_ROOT.to_vec();
        push_str(&mut preimage, &key_id);
        preimage.extend_from_slice(&verifying_key);
        preimage.extend_from_slice(&current_generation.to_be_bytes());
        let root_commitment = ContentCommitment::sha256(&preimage);
        Ok(Self {
            key_id,
            verifying_key,
            current_generation,
            root_commitment,
        })
    }

    pub fn key_id(&self) -> &str {
        &self.key_id
    }

    pub fn current_generation(&self) -> u64 {
        self.current_generation
    }

    pub fn root_commitment(&self) -> &ContentCommitment {
        &self.root_commitment
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

/// Non-deserializable positive result proving one exact provider profile is
/// signed by the supplied current trust root and valid at the qualification
/// instant. This is provider-policy evidence, never institutional authority.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QualifiedProviderExecutionProfile {
    profile: ProviderExecutionProfile,
    profile_commitment: ContentCommitment,
    trust_root_commitment: ContentCommitment,
    qualified_at_ms: i64,
}

impl QualifiedProviderExecutionProfile {
    pub fn profile(&self) -> &ProviderExecutionProfile {
        &self.profile
    }

    pub fn profile_commitment(&self) -> &ContentCommitment {
        &self.profile_commitment
    }

    pub fn trust_root_commitment(&self) -> &ContentCommitment {
        &self.trust_root_commitment
    }

    pub fn qualified_at_ms(&self) -> i64 {
        self.qualified_at_ms
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub fn qualify_provider_profile(
    signed: SignedProviderExecutionProfile,
    trust_root: &ProviderProfileTrustRoot,
    now_ms: i64,
) -> Result<QualifiedProviderExecutionProfile, QualificationError> {
    validate_timestamp(now_ms)?;
    signed.profile.validate()?;
    if signed.profile.signer_key_id != trust_root.key_id {
        return Err(QualificationError::SignerMismatch);
    }
    if signed.profile.generation != trust_root.current_generation {
        return Err(QualificationError::GenerationMismatch {
            profile: signed.profile.generation,
            current: trust_root.current_generation,
        });
    }
    if now_ms < signed.profile.not_before_ms || now_ms > signed.profile.not_after_ms {
        return Err(QualificationError::ProfileNotCurrent);
    }

    let preimage = signed.profile.canonical_preimage_v1()?;
    let verifying_key = VerifyingKey::from_bytes(&trust_root.verifying_key)
        .map_err(|_| QualificationError::InvalidVerifyingKey)?;
    let signature = Signature::from_bytes(&signed.signature);
    verifying_key
        .verify(&preimage, &signature)
        .map_err(|_| QualificationError::InvalidSignature)?;

    let profile_commitment = ContentCommitment::sha256(&preimage);
    Ok(QualifiedProviderExecutionProfile {
        profile: signed.profile,
        profile_commitment,
        trust_root_commitment: trust_root.root_commitment.clone(),
        qualified_at_ms: now_ms,
    })
}

/// Non-deserializable compatibility theorem for one exact INT-03 claim, typed
/// command, and current signed provider profile.
///
/// This object deliberately contains no provider payload bytes and cannot cross
/// the durable runtime into `DispatchStarted` by itself.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct QualifiedExecutionBinding {
    entry_id: i64,
    attempt_id: mycelix_integration_core::ExecutionAttemptId,
    command_id: IntegrationCommandId,
    connector_instance: ConnectorInstanceId,
    command_commitment: ContentCommitment,
    provider_profile_commitment: ContentCommitment,
    provider_trust_root_commitment: ContentCommitment,
    materializer_release: ContentCommitment,
    lease_until_ms: i64,
    qualified_at_ms: i64,
    binding_commitment: ContentCommitment,
}

impl QualifiedExecutionBinding {
    pub fn entry_id(&self) -> i64 {
        self.entry_id
    }

    pub fn attempt_id(&self) -> &mycelix_integration_core::ExecutionAttemptId {
        &self.attempt_id
    }

    pub fn command_id(&self) -> &IntegrationCommandId {
        &self.command_id
    }

    pub fn connector_instance(&self) -> &ConnectorInstanceId {
        &self.connector_instance
    }

    pub fn command_commitment(&self) -> &ContentCommitment {
        &self.command_commitment
    }

    pub fn provider_profile_commitment(&self) -> &ContentCommitment {
        &self.provider_profile_commitment
    }

    pub fn provider_trust_root_commitment(&self) -> &ContentCommitment {
        &self.provider_trust_root_commitment
    }

    pub fn materializer_release(&self) -> &ContentCommitment {
        &self.materializer_release
    }

    pub fn lease_until_ms(&self) -> i64 {
        self.lease_until_ms
    }

    pub fn qualified_at_ms(&self) -> i64 {
        self.qualified_at_ms
    }

    pub fn binding_commitment(&self) -> &ContentCommitment {
        &self.binding_commitment
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }

    pub const fn payload_materialized_here(&self) -> bool {
        false
    }

    pub const fn dispatch_started_here(&self) -> bool {
        false
    }
}

pub fn qualify_execution_binding<C>(
    claim: &ExecutionClaim,
    command: &IntegrationCommand<C>,
    provider: &QualifiedProviderExecutionProfile,
    now_ms: i64,
) -> Result<QualifiedExecutionBinding, QualificationError>
where
    C: CanonicalEncodeV1,
{
    validate_timestamp(now_ms)?;
    if claim.entry_id <= 0 {
        return Err(QualificationError::InvalidEntryId);
    }
    if now_ms >= claim.lease_until_ms {
        return Err(QualificationError::ClaimLeaseExpired);
    }
    if command.command_id != claim.command_id {
        return Err(QualificationError::CommandIdMismatch);
    }
    if command.connector_instance != claim.connector_instance {
        return Err(QualificationError::ConnectorMismatch);
    }
    let command_commitment = command.canonical_commitment_v1();
    if command_commitment != claim.command_commitment {
        return Err(QualificationError::CommandCommitmentMismatch);
    }
    if command.side_effect_class != claim.side_effect_class {
        return Err(QualificationError::SideEffectMismatch);
    }
    if command.idempotency_key != claim.idempotency_key {
        return Err(QualificationError::IdempotencyKeyMismatch);
    }

    let profile = provider.profile();
    if now_ms < profile.not_before_ms || now_ms > profile.not_after_ms {
        return Err(QualificationError::ProfileNotCurrent);
    }
    if profile.connector_instance != command.connector_instance {
        return Err(QualificationError::ProfileConnectorMismatch);
    }
    if profile.system != command.system {
        return Err(QualificationError::ProfileSystemMismatch);
    }
    if profile.operation_kind != command.operation_kind {
        return Err(QualificationError::ProfileOperationMismatch);
    }
    if profile.side_effect_class != command.side_effect_class {
        return Err(QualificationError::ProfileSideEffectMismatch);
    }

    if let Some(target) = &command.target
        && target.system != command.system
    {
        return Err(QualificationError::TargetSystemMismatch);
    }
    if let Some(required) = &profile.required_target_type {
        match &command.target {
            Some(target) if &target.object_type == required => {}
            _ => return Err(QualificationError::TargetTypeMismatch),
        }
    }

    if command.side_effect_class != SideEffectClass::ReadOnly
        && profile.reconciliation == ProviderReconciliationMode::None
    {
        return Err(QualificationError::ReconciliationUnavailable);
    }
    if profile.reconciliation == ProviderReconciliationMode::IdempotencyKey {
        if command.idempotency_key.is_none() {
            return Err(QualificationError::IdempotencyContractMissing);
        }
        match profile.idempotency {
            ProviderIdempotencySemantics::RequestKey {
                retention_ms,
                lookup_supported: true,
                ..
            } if retention_ms > 0 => {}
            _ => return Err(QualificationError::IdempotencyContractMissing),
        }
    }

    let mut preimage = DOMAIN_BINDING.to_vec();
    preimage.extend_from_slice(&claim.entry_id.to_be_bytes());
    push_str(&mut preimage, claim.attempt_id.as_str());
    push_str(&mut preimage, claim.command_id.as_str());
    push_str(&mut preimage, claim.connector_instance.as_str());
    push_commitment(&mut preimage, &command_commitment);
    push_commitment(&mut preimage, provider.profile_commitment());
    push_commitment(&mut preimage, provider.trust_root_commitment());
    push_commitment(&mut preimage, &profile.materializer_release);
    preimage.extend_from_slice(&claim.lease_until_ms.to_be_bytes());
    preimage.extend_from_slice(&now_ms.to_be_bytes());
    let binding_commitment = ContentCommitment::sha256(&preimage);

    Ok(QualifiedExecutionBinding {
        entry_id: claim.entry_id,
        attempt_id: claim.attempt_id.clone(),
        command_id: claim.command_id.clone(),
        connector_instance: claim.connector_instance.clone(),
        command_commitment,
        provider_profile_commitment: provider.profile_commitment().clone(),
        provider_trust_root_commitment: provider.trust_root_commitment().clone(),
        materializer_release: profile.materializer_release.clone(),
        lease_until_ms: claim.lease_until_ms,
        qualified_at_ms: now_ms,
        binding_commitment,
    })
}

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum QualificationError {
    #[error("provider profile uses the wrong protocol version")]
    WrongProtocol,
    #[error("provider profile key id is invalid")]
    InvalidKeyId,
    #[error("provider profile generation must be non-zero")]
    InvalidGeneration,
    #[error("provider profile validity window is invalid")]
    InvalidValidityWindow,
    #[error("provider profile payload limit is invalid")]
    InvalidPayloadLimit,
    #[error("provider idempotency contract is internally inconsistent")]
    InvalidIdempotencyContract,
    #[error("provider trust root contains an invalid Ed25519 public key")]
    InvalidVerifyingKey,
    #[error("provider profile signer does not match the supplied trust root")]
    SignerMismatch,
    #[error("provider profile generation {profile} is not current generation {current}")]
    GenerationMismatch { profile: u64, current: u64 },
    #[error("provider profile is not current at the requested instant")]
    ProfileNotCurrent,
    #[error("provider profile signature is invalid")]
    InvalidSignature,
    #[error("timestamp must be non-negative milliseconds")]
    InvalidTimestamp,
    #[error("runtime execution claim has an invalid entry id")]
    InvalidEntryId,
    #[error("runtime execution claim lease has expired")]
    ClaimLeaseExpired,
    #[error("runtime claim and command id differ")]
    CommandIdMismatch,
    #[error("runtime claim and command connector differ")]
    ConnectorMismatch,
    #[error("runtime claim and canonical command commitment differ")]
    CommandCommitmentMismatch,
    #[error("runtime claim and command side-effect classification differ")]
    SideEffectMismatch,
    #[error("runtime claim and command idempotency key differ")]
    IdempotencyKeyMismatch,
    #[error("provider profile connector does not match the command")]
    ProfileConnectorMismatch,
    #[error("provider profile system does not match the command")]
    ProfileSystemMismatch,
    #[error("provider profile operation kind does not match the command")]
    ProfileOperationMismatch,
    #[error("provider profile side-effect classification does not match the command")]
    ProfileSideEffectMismatch,
    #[error("command target belongs to a different external system")]
    TargetSystemMismatch,
    #[error("command target type does not satisfy the provider profile")]
    TargetTypeMismatch,
    #[error("side-effecting command lacks provider reconciliation support")]
    ReconciliationUnavailable,
    #[error("idempotency-key reconciliation lacks a qualified provider key contract")]
    IdempotencyContractMissing,
}

fn validate_key_id(value: &str) -> Result<(), QualificationError> {
    if value.is_empty()
        || value.len() > MAX_KEY_ID_BYTES
        || value.chars().any(char::is_control)
    {
        return Err(QualificationError::InvalidKeyId);
    }
    Ok(())
}

fn validate_timestamp(value: i64) -> Result<(), QualificationError> {
    if value < 0 {
        return Err(QualificationError::InvalidTimestamp);
    }
    Ok(())
}

fn side_effect_tag(value: SideEffectClass) -> u8 {
    match value {
        SideEffectClass::ReadOnly => 0,
        SideEffectClass::Reversible => 1,
        SideEffectClass::Compensatable => 2,
        SideEffectClass::Irreversible => 3,
    }
}

fn reconciliation_tag(value: ProviderReconciliationMode) -> u8 {
    match value {
        ProviderReconciliationMode::None => 0,
        ProviderReconciliationMode::ExactOperation => 1,
        ProviderReconciliationMode::IdempotencyKey => 2,
        ProviderReconciliationMode::ObjectLookup => 3,
        ProviderReconciliationMode::CursorScan => 4,
    }
}

fn push_str(out: &mut Vec<u8>, value: &str) {
    out.extend_from_slice(&(value.len() as u64).to_be_bytes());
    out.extend_from_slice(value.as_bytes());
}

fn push_commitment(out: &mut Vec<u8>, value: &ContentCommitment) {
    match value.algorithm {
        DigestAlgorithm::Sha256 => out.push(1),
    }
    out.extend_from_slice(&value.digest);
}

#[cfg(test)]
mod tests {
    use super::*;
    use ed25519_dalek::{Signer, SigningKey};
    use mycelix_integration_core::{
        ExecutionAttemptId, ExternalObjectRef, ExternalOpaqueId, ValidationError,
    };

    #[derive(Debug, Clone, PartialEq, Eq)]
    struct DemoPayload(u64);

    impl CanonicalEncodeV1 for DemoPayload {
        fn canonical_preimage_v1(&self) -> Vec<u8> {
            let mut out = b"DEMO-PAYLOAD\0V1\0".to_vec();
            out.extend_from_slice(&self.0.to_be_bytes());
            out
        }
    }

    fn id<T>(value: &str, constructor: fn(String) -> Result<T, ValidationError>) -> T {
        constructor(value.to_owned()).expect("valid fixture id")
    }

    fn signing_key() -> SigningKey {
        SigningKey::from_bytes(&[7_u8; 32])
    }

    fn command(amount: u64) -> IntegrationCommand<DemoPayload> {
        IntegrationCommand {
            command_id: id("cmd-1", IntegrationCommandId::new),
            connector_instance: id("provider-prod-1", ConnectorInstanceId::new),
            system: id("provider-x", ExternalSystemId::new),
            operation_kind: id("create-transfer", ExternalOperationKind::new),
            target: Some(ExternalObjectRef {
                system: id("provider-x", ExternalSystemId::new),
                object_type: id("account", ExternalObjectType::new),
                external_id: id("acct-1", ExternalOpaqueId::new),
            }),
            side_effect_class: SideEffectClass::Irreversible,
            idempotency_key: Some(id("idem-cmd-1", IdempotencyKey::new)),
            semantic_profile: id("transfer@1", SemanticProfileId::new),
            payload: DemoPayload(amount),
        }
    }

    fn profile(generation: u64) -> ProviderExecutionProfile {
        ProviderExecutionProfile {
            protocol_version: PROFILE_PROTOCOL_V1.to_owned(),
            profile_id: id("provider-x-transfer@1", SemanticProfileId::new),
            generation,
            signer_key_id: "root-1".to_owned(),
            connector_instance: id("provider-prod-1", ConnectorInstanceId::new),
            system: id("provider-x", ExternalSystemId::new),
            operation_kind: id("create-transfer", ExternalOperationKind::new),
            required_target_type: Some(id("account", ExternalObjectType::new)),
            side_effect_class: SideEffectClass::Irreversible,
            idempotency: ProviderIdempotencySemantics::RequestKey {
                retention_ms: 86_400_000,
                lookup_supported: true,
                duplicate_returns_original: true,
            },
            reconciliation: ProviderReconciliationMode::IdempotencyKey,
            materializer_release: ContentCommitment::sha256(b"adapter-release-v1"),
            max_payload_bytes: 64 * 1024,
            not_before_ms: 100,
            not_after_ms: 1_000,
        }
    }

    fn signed_profile(generation: u64) -> SignedProviderExecutionProfile {
        let profile = profile(generation);
        let signature = signing_key()
            .sign(&profile.canonical_preimage_v1().expect("valid profile"))
            .to_bytes();
        SignedProviderExecutionProfile { profile, signature }
    }

    fn trust_root(generation: u64) -> ProviderProfileTrustRoot {
        ProviderProfileTrustRoot::new(
            "root-1",
            signing_key().verifying_key().to_bytes(),
            generation,
        )
        .expect("valid root")
    }

    fn claim(command: &IntegrationCommand<DemoPayload>) -> ExecutionClaim {
        ExecutionClaim {
            entry_id: 7,
            attempt_id: id("7:1", ExecutionAttemptId::new),
            command_id: command.command_id.clone(),
            connector_instance: command.connector_instance.clone(),
            command_commitment: command.canonical_commitment_v1(),
            side_effect_class: command.side_effect_class,
            idempotency_key: command.idempotency_key.clone(),
            attempt_count: 1,
            lease_until_ms: 900,
        }
    }

    #[test]
    fn exact_claim_command_and_current_signed_profile_bind_without_granting_authority() {
        let command = command(5000);
        let qualified = qualify_provider_profile(signed_profile(3), &trust_root(3), 200)
            .expect("profile qualifies");
        let binding = qualify_execution_binding(&claim(&command), &command, &qualified, 300)
            .expect("exact binding qualifies");

        assert_eq!(binding.command_id(), &command.command_id);
        assert_eq!(binding.attempt_id().as_str(), "7:1");
        assert!(!qualified.grants_execution_authority());
        assert!(!binding.grants_execution_authority());
        assert!(!binding.payload_materialized_here());
        assert!(!binding.dispatch_started_here());
    }

    #[test]
    fn stale_profile_generation_fails_closed() {
        assert_eq!(
            qualify_provider_profile(signed_profile(3), &trust_root(4), 200),
            Err(QualificationError::GenerationMismatch {
                profile: 3,
                current: 4,
            })
        );
    }

    #[test]
    fn invalid_signature_fails_closed() {
        let mut signed = signed_profile(3);
        signed.signature[0] ^= 1;
        assert_eq!(
            qualify_provider_profile(signed, &trust_root(3), 200),
            Err(QualificationError::InvalidSignature)
        );
    }

    #[test]
    fn command_substitution_fails_exact_commitment_binding() {
        let original = command(5000);
        let claim = claim(&original);
        let changed = command(5001);
        let qualified = qualify_provider_profile(signed_profile(3), &trust_root(3), 200).unwrap();
        assert_eq!(
            qualify_execution_binding(&claim, &changed, &qualified, 300),
            Err(QualificationError::CommandCommitmentMismatch)
        );
    }

    #[test]
    fn expired_attempt_cannot_be_bound() {
        let command = command(5000);
        let qualified = qualify_provider_profile(signed_profile(3), &trust_root(3), 200).unwrap();
        assert_eq!(
            qualify_execution_binding(&claim(&command), &command, &qualified, 900),
            Err(QualificationError::ClaimLeaseExpired)
        );
    }

    #[test]
    fn provider_operation_substitution_fails() {
        let command = command(5000);
        let mut signed = signed_profile(3);
        signed.profile.operation_kind = id("delete-account", ExternalOperationKind::new);
        signed.signature = signing_key()
            .sign(&signed.profile.canonical_preimage_v1().unwrap())
            .to_bytes();
        let qualified = qualify_provider_profile(signed, &trust_root(3), 200).unwrap();
        assert_eq!(
            qualify_execution_binding(&claim(&command), &command, &qualified, 300),
            Err(QualificationError::ProfileOperationMismatch)
        );
    }

    #[test]
    fn idempotency_key_presence_is_not_a_provider_guarantee() {
        let mut profile = profile(3);
        profile.idempotency = ProviderIdempotencySemantics::Unsupported;
        assert_eq!(
            profile.validate(),
            Err(QualificationError::InvalidIdempotencyContract)
        );
    }

    #[test]
    fn side_effecting_command_requires_reconciliation_support() {
        let command = command(5000);
        let mut signed = signed_profile(3);
        signed.profile.reconciliation = ProviderReconciliationMode::None;
        signed.signature = signing_key()
            .sign(&signed.profile.canonical_preimage_v1().unwrap())
            .to_bytes();
        let qualified = qualify_provider_profile(signed, &trust_root(3), 200).unwrap();
        assert_eq!(
            qualify_execution_binding(&claim(&command), &command, &qualified, 300),
            Err(QualificationError::ReconciliationUnavailable)
        );
    }
}

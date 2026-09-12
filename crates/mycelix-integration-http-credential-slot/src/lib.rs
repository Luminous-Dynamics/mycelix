//! Exact-version, non-secret credential-slot qualification for HTTP integrations.
//!
//! The institution-adopted HTTP transport policy authorizes a credential slot
//! identity and header role. This crate additionally binds one exact live secret
//! manager version/handle observation to that policy without reading, returning,
//! hashing, or persisting the secret value itself.
//!
//! The eventual one-shot HTTPS engine must resolve the exact qualified version;
//! it must not ask for an unversioned "latest" secret after qualification.

use mycelix_institutional_core::Digest32;
use mycelix_integration_core::{ConnectorInstanceId, ContentCommitment, ExternalSystemId};
use mycelix_integration_http_transport_policy::QualifiedHttpTransportProviderBinding;
use thiserror::Error;

pub const CREDENTIAL_SLOT_PROFILE: &str =
    "mycelix-integration-http-credential-slot-v1-blake3-framed";
const DOMAIN_SLOT: &[u8] = b"mycelix/integration/http-credential-slot/v1";
const MAX_TEXT_BYTES: usize = 1024;

/// Metadata-only observation from a future secret-manager boundary. It is not a
/// secret and is not trusted merely because a caller can construct it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct CredentialSlotMetadataObservation {
    pub slot_id: String,
    pub version_id: String,
    pub rotation_generation: u64,
    pub connector_instance: ConnectorInstanceId,
    pub system: ExternalSystemId,
    pub provider_profile_commitment: ContentCommitment,
    pub credential_header_name: String,
    /// Stable non-secret commitment to the secret-manager object/version handle,
    /// not a digest of credential material.
    pub handle_commitment: Digest32,
    pub enabled: bool,
    pub verifier_ref: String,
    pub observed_at_ms: u64,
    pub valid_until_ms: u64,
}

/// Non-deserializable proof that one exact metadata-only secret version is the
/// current slot/version compatible with one exact HTTP transport/provider policy.
#[derive(Clone, Debug)]
pub struct QualifiedHttpCredentialSlot {
    slot_id: String,
    version_id: String,
    rotation_generation: u64,
    handle_commitment: Digest32,
    transport_policy_binding_digest: Digest32,
    qualification_digest: Digest32,
    verifier_ref: String,
    qualified_at_ms: u64,
    valid_until_ms: u64,
}

impl QualifiedHttpCredentialSlot {
    pub fn slot_id(&self) -> &str {
        &self.slot_id
    }

    pub fn version_id(&self) -> &str {
        &self.version_id
    }

    pub fn rotation_generation(&self) -> u64 {
        self.rotation_generation
    }

    pub fn handle_commitment(&self) -> Digest32 {
        self.handle_commitment
    }

    pub fn transport_policy_binding_digest(&self) -> Digest32 {
        self.transport_policy_binding_digest
    }

    pub fn qualification_digest(&self) -> Digest32 {
        self.qualification_digest
    }

    pub fn verifier_ref(&self) -> &str {
        &self.verifier_ref
    }

    pub fn qualified_at_ms(&self) -> u64 {
        self.qualified_at_ms
    }

    pub fn valid_until_ms(&self) -> u64 {
        self.valid_until_ms
    }

    pub fn is_live_at(&self, now_ms: u64) -> bool {
        now_ms >= self.qualified_at_ms && now_ms < self.valid_until_ms
    }

    pub const fn exact_version_resolution_required_here(&self) -> bool {
        true
    }

    pub const fn fallback_to_latest_allowed_here(&self) -> bool {
        false
    }

    pub const fn environment_credential_fallback_allowed_here(&self) -> bool {
        false
    }

    pub const fn credential_material_present_here(&self) -> bool {
        false
    }

    pub const fn credential_material_hashed_here(&self) -> bool {
        false
    }

    pub const fn credential_material_persisted_here(&self) -> bool {
        false
    }

    pub const fn metadata_origin_verified_here(&self) -> bool {
        false
    }

    pub const fn secret_manager_access_performed_here(&self) -> bool {
        false
    }

    pub const fn grants_execution_authority(&self) -> bool {
        false
    }
}

pub fn qualify_http_credential_slot(
    transport: &QualifiedHttpTransportProviderBinding,
    observation: CredentialSlotMetadataObservation,
    now_ms: u64,
) -> Result<QualifiedHttpCredentialSlot, CredentialSlotError> {
    if now_ms == 0 || transport.valid_until_ms() <= now_ms {
        return Err(CredentialSlotError::TransportPolicyNotLive);
    }
    let policy = transport.current_policy().policy();
    validate_text(&observation.slot_id)?;
    validate_text(&observation.version_id)?;
    validate_text(&observation.verifier_ref)?;
    validate_text(&observation.credential_header_name)?;
    if observation.slot_id != policy.credential_slot_id {
        return Err(CredentialSlotError::SlotMismatch);
    }
    if observation.credential_header_name != policy.credential_header_name {
        return Err(CredentialSlotError::HeaderRoleMismatch);
    }
    if observation.connector_instance != policy.connector_instance
        || observation.system != policy.system
    {
        return Err(CredentialSlotError::ProviderScopeMismatch);
    }
    if &observation.provider_profile_commitment != transport.provider_profile_commitment() {
        return Err(CredentialSlotError::ProviderProfileMismatch);
    }
    if !observation.enabled {
        return Err(CredentialSlotError::CredentialDisabled);
    }
    if observation.rotation_generation == 0 {
        return Err(CredentialSlotError::InvalidRotationGeneration);
    }
    if observation.handle_commitment.0 == [0; 32] {
        return Err(CredentialSlotError::ZeroHandleCommitment);
    }
    if observation.observed_at_ms == 0
        || observation.observed_at_ms > now_ms
        || observation.valid_until_ms <= now_ms
        || observation.valid_until_ms <= observation.observed_at_ms
    {
        return Err(CredentialSlotError::InvalidObservationWindow);
    }

    let valid_until_ms = observation.valid_until_ms.min(transport.valid_until_ms());
    if valid_until_ms <= now_ms {
        return Err(CredentialSlotError::NoUsableCredentialWindow);
    }
    let qualification_digest = slot_digest(
        transport.binding_digest(),
        &observation.slot_id,
        &observation.version_id,
        observation.rotation_generation,
        observation.handle_commitment,
        &observation.credential_header_name,
        &observation.verifier_ref,
        observation.observed_at_ms,
        valid_until_ms,
    );

    Ok(QualifiedHttpCredentialSlot {
        slot_id: observation.slot_id,
        version_id: observation.version_id,
        rotation_generation: observation.rotation_generation,
        handle_commitment: observation.handle_commitment,
        transport_policy_binding_digest: transport.binding_digest(),
        qualification_digest,
        verifier_ref: observation.verifier_ref,
        qualified_at_ms: now_ms,
        valid_until_ms,
    })
}

fn slot_digest(
    transport_binding: Digest32,
    slot_id: &str,
    version_id: &str,
    rotation_generation: u64,
    handle_commitment: Digest32,
    header_name: &str,
    verifier_ref: &str,
    observed_at_ms: u64,
    valid_until_ms: u64,
) -> Digest32 {
    let mut h = blake3::Hasher::new();
    h.update(DOMAIN_SLOT);
    frame(&mut h, CREDENTIAL_SLOT_PROFILE.as_bytes());
    frame(&mut h, &transport_binding.0);
    frame(&mut h, slot_id.as_bytes());
    frame(&mut h, version_id.as_bytes());
    frame(&mut h, &rotation_generation.to_le_bytes());
    frame(&mut h, &handle_commitment.0);
    frame(&mut h, header_name.as_bytes());
    frame(&mut h, verifier_ref.as_bytes());
    frame(&mut h, &observed_at_ms.to_le_bytes());
    frame(&mut h, &valid_until_ms.to_le_bytes());
    Digest32(*h.finalize().as_bytes())
}

fn validate_text(value: &str) -> Result<(), CredentialSlotError> {
    if value.trim().is_empty()
        || value.len() > MAX_TEXT_BYTES
        || value.chars().any(char::is_control)
    {
        Err(CredentialSlotError::InvalidText)
    } else {
        Ok(())
    }
}

fn frame(h: &mut blake3::Hasher, bytes: &[u8]) {
    h.update(&(bytes.len() as u64).to_le_bytes());
    h.update(bytes);
}

#[derive(Debug, Error, PartialEq, Eq)]
pub enum CredentialSlotError {
    #[error("current HTTP transport policy is not live")]
    TransportPolicyNotLive,
    #[error("credential-slot metadata contains invalid text")]
    InvalidText,
    #[error("credential slot differs from institution-adopted HTTP policy")]
    SlotMismatch,
    #[error("credential header role differs from institution-adopted HTTP policy")]
    HeaderRoleMismatch,
    #[error("credential connector/system scope differs from HTTP policy")]
    ProviderScopeMismatch,
    #[error("credential slot is bound to a different provider profile")]
    ProviderProfileMismatch,
    #[error("credential version is disabled/revoked")]
    CredentialDisabled,
    #[error("credential rotation generation must be non-zero")]
    InvalidRotationGeneration,
    #[error("credential version handle commitment must be non-zero")]
    ZeroHandleCommitment,
    #[error("credential metadata observation window is invalid")]
    InvalidObservationWindow,
    #[error("no usable credential qualification window remains")]
    NoUsableCredentialWindow,
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn text_validation_rejects_empty_and_control_data() {
        assert_eq!(validate_text(""), Err(CredentialSlotError::InvalidText));
        assert_eq!(validate_text("bad\nslot"), Err(CredentialSlotError::InvalidText));
        assert!(validate_text("provider-api-key/current").is_ok());
    }

    #[test]
    fn slot_digest_changes_with_exact_version_and_handle() {
        let a = slot_digest(
            Digest32([1; 32]),
            "slot-a",
            "version-1",
            1,
            Digest32([2; 32]),
            "authorization",
            "verifier-a",
            10,
            20,
        );
        let b = slot_digest(
            Digest32([1; 32]),
            "slot-a",
            "version-2",
            2,
            Digest32([3; 32]),
            "authorization",
            "verifier-a",
            10,
            20,
        );
        assert_ne!(a, b);
    }
}

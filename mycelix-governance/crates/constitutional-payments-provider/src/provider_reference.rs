#![deny(unsafe_code)]

//! Inert semantic provider-reference identity boundary.
//!
//! This module deliberately does not authorize, dispatch, reconcile, or qualify a
//! provider. It gives later adapters a narrow type boundary for separating:
//!
//!   native replay identity + material action + effecting target + execution domain
//!       -> semantic provider-reference seed
//!       -> carriage descriptor
//!       -> provider wire reference
//!
//! The semantic seed excludes operation IDs, attempt IDs, retry nonces, profile labels,
//! adapter implementation digests, and qualification evidence by construction: those
//! values are not accepted by the seed constructor.

use blake3::Hasher;
use thiserror::Error;

pub const PROVIDER_REFERENCE_SEED_SCHEMA_VERSION: u16 = 1;
pub const PROVIDER_REFERENCE_SEED_PREFIX: &str = "payments-provider-reference-seed-v1:";
pub const PROVIDER_REFERENCE_DESCRIPTOR_PREFIX: &str =
    "payments-provider-reference-descriptor-v1:";
const MAX_REF_LEN: usize = 512;
const MAX_PREFIX_LEN: usize = 128;
const SEED_DOMAIN: &[u8] =
    b"MYCELIX-CONSTITUTIONAL-PAYMENTS-PROVIDER-REFERENCE-SEED\0V1\0";
const DESCRIPTOR_DOMAIN: &[u8] =
    b"MYCELIX-CONSTITUTIONAL-PAYMENTS-PROVIDER-REFERENCE-DESCRIPTOR\0V1\0";
const WIRE_DOMAIN: &[u8] =
    b"MYCELIX-CONSTITUTIONAL-PAYMENTS-PROVIDER-REFERENCE-WIRE\0V1\0";

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProviderReferenceError {
    Violation(String),
}

impl std::fmt::Display for ProviderReferenceError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Violation(message) => f.write_str(message),
        }
    }
}

impl std::error::Error for ProviderReferenceError {}

pub type ProviderReferenceResult<T> = Result<T, ProviderReferenceError>;

fn violation(message: impl Into<String>) -> ProviderReferenceError {
    ProviderReferenceError::Violation(message.into())
}

fn require_opaque(label: &str, value: &str, max_len: usize) -> ProviderReferenceResult<()> {
    if value.trim().is_empty() || value.len() > max_len {
        return Err(violation(format!(
            "{label} must be non-empty and <= {max_len} bytes"
        )));
    }
    Ok(())
}

fn push_str(hasher: &mut Hasher, value: &str) {
    let bytes = value.as_bytes();
    hasher.update(&(bytes.len() as u64).to_be_bytes());
    hasher.update(bytes);
}

fn tagged(prefix: &str, hash: blake3::Hash) -> String {
    format!("{prefix}{}", hash.to_hex())
}

fn require_tagged_hex(
    label: &str,
    value: &str,
    prefix: &str,
) -> ProviderReferenceResult<()> {
    let digest = value
        .strip_prefix(prefix)
        .ok_or_else(|| violation(format!("{label} must use {prefix}<hex>")))?;
    if digest.len() != 64
        || !digest
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
    {
        return Err(violation(format!(
            "{label} must contain exactly 64 lowercase hexadecimal digits"
        )));
    }
    Ok(())
}

/// Semantic identity for one provider reference.
///
/// This type intentionally has no Serialize/Deserialize implementation and all
/// fields are private. It can only be built from the five semantic inputs admitted by
/// the v1 contract.
///
/// The caller is responsible for supplying an already-verified native replay identity,
/// frozen material action digest, effecting-target identity, and execution-domain
/// identity. This type does not manufacture or authenticate those upstream facts.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct ProviderReferenceSeedV1 {
    native_replay_identity: String,
    material_action_digest: String,
    effecting_target_identity: String,
    execution_domain_identity: String,
    derivation_version: u16,
    digest: String,
}

impl ProviderReferenceSeedV1 {
    pub fn new(
        native_replay_identity: impl Into<String>,
        material_action_digest: impl Into<String>,
        effecting_target_identity: impl Into<String>,
        execution_domain_identity: impl Into<String>,
        derivation_version: u16,
    ) -> ProviderReferenceResult<Self> {
        if derivation_version == 0 {
            return Err(violation("derivation_version must be non-zero"));
        }

        let native_replay_identity = native_replay_identity.into();
        let material_action_digest = material_action_digest.into();
        let effecting_target_identity = effecting_target_identity.into();
        let execution_domain_identity = execution_domain_identity.into();

        require_opaque(
            "native_replay_identity",
            &native_replay_identity,
            MAX_REF_LEN,
        )?;
        require_opaque(
            "material_action_digest",
            &material_action_digest,
            MAX_REF_LEN,
        )?;
        require_opaque(
            "effecting_target_identity",
            &effecting_target_identity,
            MAX_REF_LEN,
        )?;
        require_opaque(
            "execution_domain_identity",
            &execution_domain_identity,
            MAX_REF_LEN,
        )?;

        let mut out = Self {
            native_replay_identity,
            material_action_digest,
            effecting_target_identity,
            execution_domain_identity,
            derivation_version,
            digest: String::new(),
        };
        out.digest = out.compute_digest();
        Ok(out)
    }

    fn compute_digest(&self) -> String {
        let mut hasher = Hasher::new();
        hasher.update(SEED_DOMAIN);
        hasher.update(&PROVIDER_REFERENCE_SEED_SCHEMA_VERSION.to_be_bytes());
        hasher.update(&self.derivation_version.to_be_bytes());
        push_str(&mut hasher, &self.native_replay_identity);
        push_str(&mut hasher, &self.material_action_digest);
        push_str(&mut hasher, &self.effecting_target_identity);
        push_str(&mut hasher, &self.execution_domain_identity);
        tagged(PROVIDER_REFERENCE_SEED_PREFIX, hasher.finalize())
    }

    pub fn digest(&self) -> &str {
        &self.digest
    }

    pub fn native_replay_identity(&self) -> &str {
        &self.native_replay_identity
    }

    pub fn material_action_digest(&self) -> &str {
        &self.material_action_digest
    }

    pub fn effecting_target_identity(&self) -> &str {
        &self.effecting_target_identity
    }

    pub fn execution_domain_identity(&self) -> &str {
        &self.execution_domain_identity
    }

    pub fn derivation_version(&self) -> u16 {
        self.derivation_version
    }

    /// Build the non-semantic carriage descriptor.
    ///
    /// Profile identity and carriage algorithm version can rotate without changing
    /// the semantic seed. They affect the actual provider reference only after this
    /// boundary, never the seed itself.
    pub fn carriage_descriptor(
        &self,
        provider_profile_identity_digest: impl Into<String>,
        carriage_algorithm_version: u16,
    ) -> ProviderReferenceResult<ProviderReferenceDescriptorV1> {
        if carriage_algorithm_version == 0 {
            return Err(violation(
                "carriage_algorithm_version must be non-zero",
            ));
        }
        let provider_profile_identity_digest = provider_profile_identity_digest.into();
        require_opaque(
            "provider_profile_identity_digest",
            &provider_profile_identity_digest,
            MAX_REF_LEN,
        )?;

        ProviderReferenceDescriptorV1::new(
            self.digest.clone(),
            provider_profile_identity_digest,
            carriage_algorithm_version,
        )
    }
}

/// A carriage-specific descriptor derived from one semantic seed.
///
/// This value is intentionally distinct from the seed. Rotating an adapter/profile or
/// its carriage algorithm can therefore change the wire reference while preserving
/// semantic provider identity.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct ProviderReferenceDescriptorV1 {
    seed_digest: String,
    provider_profile_identity_digest: String,
    carriage_algorithm_version: u16,
    digest: String,
}

impl ProviderReferenceDescriptorV1 {
    fn new(
        seed_digest: String,
        provider_profile_identity_digest: String,
        carriage_algorithm_version: u16,
    ) -> ProviderReferenceResult<Self> {
        require_tagged_hex(
            "seed_digest",
            &seed_digest,
            PROVIDER_REFERENCE_SEED_PREFIX,
        )?;
        require_opaque(
            "provider_profile_identity_digest",
            &provider_profile_identity_digest,
            MAX_REF_LEN,
        )?;

        let mut out = Self {
            seed_digest,
            provider_profile_identity_digest,
            carriage_algorithm_version,
            digest: String::new(),
        };
        out.digest = out.compute_digest();
        Ok(out)
    }

    fn compute_digest(&self) -> String {
        let mut hasher = Hasher::new();
        hasher.update(DESCRIPTOR_DOMAIN);
        hasher.update(&PROVIDER_REFERENCE_SEED_SCHEMA_VERSION.to_be_bytes());
        push_str(&mut hasher, &self.seed_digest);
        push_str(&mut hasher, &self.provider_profile_identity_digest);
        hasher.update(&self.carriage_algorithm_version.to_be_bytes());
        tagged(PROVIDER_REFERENCE_DESCRIPTOR_PREFIX, hasher.finalize())
    }

    pub fn seed_digest(&self) -> &str {
        &self.seed_digest
    }

    pub fn provider_profile_identity_digest(&self) -> &str {
        &self.provider_profile_identity_digest
    }

    pub fn carriage_algorithm_version(&self) -> u16 {
        self.carriage_algorithm_version
    }

    pub fn digest(&self) -> &str {
        &self.digest
    }

    /// Render a deterministic provider-facing reference under a carriage prefix.
    ///
    /// The prefix is wire syntax only. It is deliberately excluded from the
    /// descriptor digest and therefore cannot change semantic or carriage identity.
    /// Provider-specific length/character requirements remain a separate qualification
    /// concern for the adapter.
    pub fn render_reference(
        &self,
        wire_prefix: impl Into<String>,
    ) -> ProviderReferenceResult<String> {
        let wire_prefix = wire_prefix.into();
        require_opaque("wire_prefix", &wire_prefix, MAX_PREFIX_LEN)?;

        let mut hasher = Hasher::new();
        hasher.update(WIRE_DOMAIN);
        push_str(&mut hasher, &self.digest);
        let suffix = hasher.finalize().to_hex();

        Ok(format!("{wire_prefix}{suffix}"))
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn seed(
        replay: &str,
        action: &str,
        target: &str,
        domain: &str,
        version: u16,
    ) -> ProviderReferenceSeedV1 {
        ProviderReferenceSeedV1::new(replay, action, target, domain, version).unwrap()
    }

    #[test]
    fn semantic_seed_is_deterministic() {
        let a = seed("grant-1", "action-1", "acct-1", "stripe-live-acct-1", 1);
        let b = seed("grant-1", "action-1", "acct-1", "stripe-live-acct-1", 1);
        assert_eq!(a, b);
        assert_eq!(a.digest(), b.digest());
    }

    #[test]
    fn profile_rotation_preserves_semantic_seed() {
        let a = seed("grant-1", "action-1", "acct-1", "stripe-live-acct-1", 1);
        let a_profile = a.carriage_descriptor("profile-integrity-a", 1).unwrap();
        let b_profile = a.carriage_descriptor("profile-integrity-b", 1).unwrap();

        assert_eq!(a_profile.seed_digest(), a.digest());
        assert_eq!(b_profile.seed_digest(), a.digest());
        assert_ne!(a_profile.digest(), b_profile.digest());
    }

    #[test]
    fn carriage_algorithm_rotation_preserves_semantic_seed() {
        let seed = seed("grant-1", "action-1", "acct-1", "stripe-live-acct-1", 1);
        let a = seed.carriage_descriptor("profile-integrity-a", 1).unwrap();
        let b = seed.carriage_descriptor("profile-integrity-a", 2).unwrap();

        assert_eq!(a.seed_digest(), b.seed_digest());
        assert_ne!(a.digest(), b.digest());
    }

    #[test]
    fn wire_prefix_rotation_does_not_change_descriptor_identity() {
        let seed = seed("grant-1", "action-1", "acct-1", "stripe-live-acct-1", 1);
        let descriptor = seed.carriage_descriptor("profile-integrity-a", 1).unwrap();

        let a = descriptor.render_reference("stripe:");
        let b = descriptor.render_reference("payments:");
        assert_ne!(a.unwrap(), b.unwrap());
        assert_eq!(descriptor.digest(), descriptor.digest());
    }

    #[test]
    fn replay_identity_rotation_changes_seed() {
        let a = seed("grant-1", "action-1", "acct-1", "stripe-live-acct-1", 1);
        let b = seed("grant-2", "action-1", "acct-1", "stripe-live-acct-1", 1);
        assert_ne!(a.digest(), b.digest());
    }

    #[test]
    fn action_rotation_changes_seed() {
        let a = seed("grant-1", "action-1", "acct-1", "stripe-live-acct-1", 1);
        let b = seed("grant-1", "action-2", "acct-1", "stripe-live-acct-1", 1);
        assert_ne!(a.digest(), b.digest());
    }

    #[test]
    fn effecting_target_rotation_changes_seed() {
        let a = seed("grant-1", "action-1", "acct-1", "stripe-live-acct-1", 1);
        let b = seed("grant-1", "action-1", "acct-2", "stripe-live-acct-1", 1);
        assert_ne!(a.digest(), b.digest());
    }

    #[test]
    fn execution_domain_rotation_changes_seed() {
        let a = seed("grant-1", "action-1", "acct-1", "stripe-live-acct-1", 1);
        let b = seed("grant-1", "action-1", "acct-1", "stripe-test-acct-1", 1);
        assert_ne!(a.digest(), b.digest());
    }

    #[test]
    fn derivation_version_rotation_changes_seed() {
        let a = seed("grant-1", "action-1", "acct-1", "stripe-live-acct-1", 1);
        let b = seed("grant-1", "action-1", "acct-1", "stripe-live-acct-1", 2);
        assert_ne!(a.digest(), b.digest());
    }

    #[test]
    fn operational_baggage_has_no_seed_slot() {
        let seed = seed("grant-1", "action-1", "acct-1", "stripe-live-acct-1", 1);
        let descriptor = seed.carriage_descriptor("profile-integrity-a", 1).unwrap();

        // Compile-time API boundary: operation_id, attempt_id, retry nonce, profile label,
        // adapter digest, and qualification evidence are not accepted by the seed API.
        assert_eq!(descriptor.seed_digest(), seed.digest());
    }

    #[test]
    fn tagged_digests_are_reproducible() {
        let seed = seed("grant-1", "action-1", "acct-1", "stripe-live-acct-1", 1);
        assert!(seed.digest().starts_with(PROVIDER_REFERENCE_SEED_PREFIX));
        assert_eq!(seed.digest().len(), PROVIDER_REFERENCE_SEED_PREFIX.len() + 64);

        let descriptor = seed.carriage_descriptor("profile-integrity-a", 1).unwrap();
        assert!(descriptor
            .digest()
            .starts_with(PROVIDER_REFERENCE_DESCRIPTOR_PREFIX));
        assert_eq!(
            descriptor.digest().len(),
            PROVIDER_REFERENCE_DESCRIPTOR_PREFIX.len() + 64
        );
    }

    #[test]
    fn semantic_seed_uses_unambiguous_length_prefixing() {
        let a = seed("ab", "c", "target", "domain", 1);
        let b = seed("a", "bc", "target", "domain", 1);
        assert_ne!(a.digest(), b.digest());
    }

    #[test]
    fn zero_versions_are_rejected() {
        assert!(ProviderReferenceSeedV1::new("grant", "action", "target", "domain", 0).is_err());
        let seed = seed("grant", "action", "target", "domain", 1);
        assert!(seed.carriage_descriptor("profile", 0).is_err());
    }

    #[test]
    fn invalid_seed_descriptor_is_rejected() {
        assert!(ProviderReferenceDescriptorV1::new("not-a-seed", "profile", 1).is_err());
    }
}

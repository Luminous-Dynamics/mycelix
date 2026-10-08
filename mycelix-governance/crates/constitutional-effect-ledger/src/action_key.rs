#![deny(unsafe_code)]

//! Inert identity contracts for the constitutional same-action fence.
//!
//! Two namespaces are deliberately separate:
//!
//! ActionKeyV1:
//!   relying party + effecting target + material action digest
//!
//! AttemptIdentityV1:
//!   boundary kind + boundary instance + attempt identifier
//!
//! Neither includes an operation identifier. Operation identifiers can remain ordinary
//! attempt metadata without becoming the shared collision namespace.

use blake3::Hasher;

pub const ACTION_KEY_SCHEMA_VERSION: u16 = 1;
pub const ACTION_KEY_PREFIX: &str = "constitutional-action-key-v1:";
pub const ATTEMPT_IDENTITY_SCHEMA_VERSION: u16 = 1;
pub const ATTEMPT_IDENTITY_PREFIX: &str = "constitutional-attempt-identity-v1:";
pub const MATERIAL_ACTION_DIGEST_PREFIX: &str = "constitutional-material-action-v1:";
pub const EXECUTION_AUTHORIZATION_SCHEMA_VERSION: u16 = 2;

const MATERIAL_ACTION_DIGEST_DOMAIN: &[u8] =
    b"MYCELIX-CONSTITUTIONAL-MATERIAL-ACTION\0V1\0";

const EXECUTION_AUTHORIZATION_DOMAIN: &[u8] =
    b"MYCELIX-GOVERNANCE-EXECUTION-AUTHORIZATION\0V2\0";

const MAX_REF_LEN: usize = 512;
const ACTION_KEY_DOMAIN: &[u8] =
    b"MYCELIX-CONSTITUTIONAL-SAME-ACTION-KEY\0V1\0";
const ATTEMPT_IDENTITY_DOMAIN: &[u8] =
    b"MYCELIX-CONSTITUTIONAL-ATTEMPT-IDENTITY\0V1\0";

fn require_opaque(label: &str, value: &str) -> Result<(), String> {
    if value.trim().is_empty() || value.len() > MAX_REF_LEN {
        return Err(format!("{label} must be non-empty and <= {MAX_REF_LEN} bytes"));
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

/// Derive the canonical material action digest from the exact frozen action bytes.
///
/// The byte representation supplied here is already the executor's canonical material
/// representation. The helper intentionally has no operation, attempt, provider, or
/// caller-selected retry input.
pub fn material_action_digest(action_bytes: &[u8]) -> String {
    let mut h = Hasher::new();
    h.update(MATERIAL_ACTION_DIGEST_DOMAIN);
    h.update(&(action_bytes.len() as u64).to_be_bytes());
    h.update(action_bytes);
    tagged(MATERIAL_ACTION_DIGEST_PREFIX, h.finalize())
}

/// Derive the authorization message that a native threshold-signing path must bind to.
///
/// This is not the native replay identity. It is the exact message hash used to
/// cryptographically verify that the accepted proposal authorization covers the
/// executor-derived ActionKey.
pub fn execution_authorization_digest(
    proposal_id: &str,
    action_key_digest: &str,
) -> Vec<u8> {
    let mut h = Hasher::new();
    h.update(EXECUTION_AUTHORIZATION_DOMAIN);
    h.update(&EXECUTION_AUTHORIZATION_SCHEMA_VERSION.to_be_bytes());
    push_str(&mut h, proposal_id);
    push_str(&mut h, action_key_digest);
    h.finalize().as_bytes().to_vec()
}

/// Shared collision namespace for one material action at one effecting target.
///
/// This is the identity used by a durable same-action fence. It intentionally does not
/// contain operation_id, native replay identity, attempt_id, wrapper digest, or retry
/// metadata. A new authorization or operation therefore cannot evade an occupied fence
/// when the material action and target are unchanged.
///
/// The constructor does not authenticate the supplied facts; they must already have
/// been derived and frozen by the effect boundary.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct ActionKeyV1 {
    relying_party_identity: String,
    effecting_target_identity: String,
    material_action_digest: String,
    digest: String,
}

impl ActionKeyV1 {
    pub fn new(
        relying_party_identity: impl Into<String>,
        effecting_target_identity: impl Into<String>,
        material_action_digest: impl Into<String>,
    ) -> Result<Self, String> {
        let relying_party_identity = relying_party_identity.into();
        let effecting_target_identity = effecting_target_identity.into();
        let material_action_digest = material_action_digest.into();

        require_opaque("relying_party_identity", &relying_party_identity)?;
        require_opaque("effecting_target_identity", &effecting_target_identity)?;
        require_opaque("material_action_digest", &material_action_digest)?;

        let mut out = Self {
            relying_party_identity,
            effecting_target_identity,
            material_action_digest,
            digest: String::new(),
        };
        out.digest = out.compute_digest();
        Ok(out)
    }

    fn compute_digest(&self) -> String {
        let mut h = Hasher::new();
        h.update(ACTION_KEY_DOMAIN);
        h.update(&ACTION_KEY_SCHEMA_VERSION.to_be_bytes());
        push_str(&mut h, &self.relying_party_identity);
        push_str(&mut h, &self.effecting_target_identity);
        push_str(&mut h, &self.material_action_digest);
        tagged(ACTION_KEY_PREFIX, h.finalize())
    }

    pub fn relying_party_identity(&self) -> &str {
        &self.relying_party_identity
    }

    pub fn effecting_target_identity(&self) -> &str {
        &self.effecting_target_identity
    }

    pub fn material_action_digest(&self) -> &str {
        &self.material_action_digest
    }

    pub fn digest(&self) -> &str {
        &self.digest
    }
}

/// Unique identity for one attempt record inside one boundary kind/instance.
///
/// This is deliberately a different namespace from ActionKeyV1. The same action may
/// have multiple historical attempts across time, while an individual attempt record
/// must be unambiguously owned by exactly one boundary namespace.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct AttemptIdentityV1 {
    boundary_kind: String,
    boundary_instance_id: String,
    attempt_id: String,
    digest: String,
}

impl AttemptIdentityV1 {
    pub fn new(
        boundary_kind: impl Into<String>,
        boundary_instance_id: impl Into<String>,
        attempt_id: impl Into<String>,
    ) -> Result<Self, String> {
        let boundary_kind = boundary_kind.into();
        let boundary_instance_id = boundary_instance_id.into();
        let attempt_id = attempt_id.into();

        require_opaque("boundary_kind", &boundary_kind)?;
        require_opaque("boundary_instance_id", &boundary_instance_id)?;
        require_opaque("attempt_id", &attempt_id)?;

        let mut out = Self {
            boundary_kind,
            boundary_instance_id,
            attempt_id,
            digest: String::new(),
        };
        out.digest = out.compute_digest();
        Ok(out)
    }

    fn compute_digest(&self) -> String {
        let mut h = Hasher::new();
        h.update(ATTEMPT_IDENTITY_DOMAIN);
        h.update(&ATTEMPT_IDENTITY_SCHEMA_VERSION.to_be_bytes());
        push_str(&mut h, &self.boundary_kind);
        push_str(&mut h, &self.boundary_instance_id);
        push_str(&mut h, &self.attempt_id);
        tagged(ATTEMPT_IDENTITY_PREFIX, h.finalize())
    }

    pub fn boundary_kind(&self) -> &str {
        &self.boundary_kind
    }

    pub fn boundary_instance_id(&self) -> &str {
        &self.boundary_instance_id
    }

    pub fn attempt_id(&self) -> &str {
        &self.attempt_id
    }

    pub fn digest(&self) -> &str {
        &self.digest
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key(rp: &str, target: &str, action: &str) -> ActionKeyV1 {
        ActionKeyV1::new(rp, target, action).unwrap()
    }

    fn attempt(boundary: &str, instance: &str, attempt: &str) -> AttemptIdentityV1 {
        AttemptIdentityV1::new(boundary, instance, attempt).unwrap()
    }

    #[test]
    fn material_action_digest_is_deterministic_and_domain_separated() {
        let a = material_action_digest(br#"[{"type":"EmitEvent","event":"x"}]"#);
        let b = material_action_digest(br#"[{"type":"EmitEvent","event":"x"}]"#);
        let c = material_action_digest(br#"[{"type":"EmitEvent","event":"y"}]"#);

        assert_eq!(a, b);
        assert_ne!(a, c);
        assert!(a.starts_with(MATERIAL_ACTION_DIGEST_PREFIX));
    }

    #[test]
    fn execution_authorization_digest_binds_exact_proposal_and_action_key() {
        let a = execution_authorization_digest(
            "proposal-1",
            "constitutional-action-key-v1:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
        );
        let b = execution_authorization_digest(
            "proposal-2",
            "constitutional-action-key-v1:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
        );
        let c = execution_authorization_digest(
            "proposal-1",
            "constitutional-action-key-v1:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
        );

        assert_eq!(a.len(), 32);
        assert_ne!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn action_key_is_deterministic() {
        let a = key("rp-1", "stripe-live-account-1", "action-digest-1");
        let b = key("rp-1", "stripe-live-account-1", "action-digest-1");
        assert_eq!(a, b);
        assert_eq!(a.digest(), b.digest());
    }

    #[test]
    fn action_key_changes_with_relying_party() {
        assert_ne!(
            key("rp-1", "target", "action").digest(),
            key("rp-2", "target", "action").digest()
        );
    }

    #[test]
    fn action_key_changes_with_effecting_target() {
        assert_ne!(
            key("rp", "target-1", "action").digest(),
            key("rp", "target-2", "action").digest()
        );
    }

    #[test]
    fn action_key_changes_with_material_action() {
        assert_ne!(
            key("rp", "target", "action-1").digest(),
            key("rp", "target", "action-2").digest()
        );
    }

    #[test]
    fn action_key_has_no_attempt_or_operation_slot() {
        let a = key("rp", "target", "action");
        let b = key("rp", "target", "action");
        assert_eq!(a.digest(), b.digest());

        // Compile-time API boundary: operation_id, attempt_id, retry nonce, native replay
        // identity, and wrapper/qualification metadata are not constructor inputs.
    }

    #[test]
    fn action_key_uses_unambiguous_length_prefixing() {
        let a = key("ab", "c", "action");
        let b = key("a", "bc", "action");
        assert_ne!(a.digest(), b.digest());
    }

    #[test]
    fn attempt_identity_is_deterministic() {
        let a = attempt("payments", "boundary-1", "attempt-1");
        let b = attempt("payments", "boundary-1", "attempt-1");
        assert_eq!(a, b);
        assert_eq!(a.digest(), b.digest());
    }

    #[test]
    fn attempt_identity_changes_with_attempt_id() {
        assert_ne!(
            attempt("payments", "boundary-1", "attempt-1").digest(),
            attempt("payments", "boundary-1", "attempt-2").digest()
        );
    }

    #[test]
    fn attempt_identity_changes_with_boundary_instance() {
        assert_ne!(
            attempt("payments", "boundary-1", "attempt-1").digest(),
            attempt("payments", "boundary-2", "attempt-1").digest()
        );
    }

    #[test]
    fn attempt_identity_changes_with_boundary_kind() {
        assert_ne!(
            attempt("payments", "boundary-1", "attempt-1").digest(),
            attempt("governance", "boundary-1", "attempt-1").digest()
        );
    }

    #[test]
    fn different_attempts_can_share_one_action_key() {
        let action = key("rp", "target", "action");
        let a = attempt("payments", "boundary-1", "attempt-1");
        let b = attempt("payments", "boundary-1", "attempt-2");

        assert_ne!(a, b);
        assert_eq!(action.material_action_digest(), "action");
    }

    #[test]
    fn identity_namespaces_are_domain_separated() {
        let action = key("rp", "target", "action");
        let attempt = attempt("payments", "boundary-1", "attempt-1");

        assert_ne!(action.digest(), attempt.digest());
        assert!(action.digest().starts_with(ACTION_KEY_PREFIX));
        assert!(attempt.digest().starts_with(ATTEMPT_IDENTITY_PREFIX));
    }

    #[test]
    fn invalid_inputs_are_rejected() {
        assert!(ActionKeyV1::new("", "target", "action").is_err());
        assert!(ActionKeyV1::new("rp", "", "action").is_err());
        assert!(ActionKeyV1::new("rp", "target", "").is_err());

        assert!(AttemptIdentityV1::new("", "boundary", "attempt").is_err());
        assert!(AttemptIdentityV1::new("kind", "", "attempt").is_err());
        assert!(AttemptIdentityV1::new("kind", "boundary", "").is_err());
    }
}

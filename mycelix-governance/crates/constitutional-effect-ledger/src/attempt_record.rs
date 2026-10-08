#![deny(unsafe_code)]

//! Typed durable-attempt and same-action fence contracts.
//!
//! This module deliberately separates:
//!   * attempt ownership,
//!   * operation metadata,
//!   * native replay identity,
//!   * material action identity,
//!   * same-action collision identity,
//!   * provider-reference carriage identity, and
//!   * provider execution environment.
//!
//! The storage model below is a deterministic reference model for the required
//! atomic state transition. A production adapter must provide the same semantics
//! from a durable, shared, conflict-detecting store; this crate does not claim
//! that an in-memory map is durable.

use crate::{ActionKeyV1, AttemptIdentityV1};
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;

pub const ATTEMPT_RECORD_SCHEMA_VERSION: u16 = 5;
pub const ACTION_FENCE_SCHEMA_VERSION: u16 = 1;
pub const ATTEMPT_RECORD_PREFIX: &str = "constitutional-attempt-record-v1:";
pub const ACTION_FENCE_RECORD_PREFIX: &str = "constitutional-action-fence-v1:";
pub const NATIVE_REPLAY_BINDING_SCHEMA_VERSION: u16 = 1;
pub const NATIVE_REPLAY_BINDING_PREFIX: &str =
    "constitutional-native-replay-binding-v1:";
const MAX_ID_LEN: usize = 256;
const MAX_REF_LEN: usize = 512;
const ATTEMPT_RECORD_DOMAIN: &[u8] =
    b"MYCELIX-CONSTITUTIONAL-ATTEMPT-RECORD\0V1\0";
const ACTION_FENCE_RECORD_DOMAIN: &[u8] =
    b"MYCELIX-CONSTITUTIONAL-ACTION-FENCE-RECORD\0V1\0";
const PROVIDER_IDEMPOTENCY_KEY_DOMAIN: &[u8] =
    b"MYCELIX-CONSTITUTIONAL-PROVIDER-IDEMPOTENCY-KEY\0V1\0";
pub const PROVIDER_IDEMPOTENCY_KEY_PREFIX: &str =
    "constitutional-provider-idempotency-v1:";

fn valid_opaque(value: &str, max_len: usize) -> bool {
    !value.trim().is_empty() && value.len() <= max_len
}

fn require_opaque(label: &str, value: &str, max_len: usize) -> Result<(), String> {
    if valid_opaque(value, max_len) {
        Ok(())
    } else {
        Err(format!("{label} must be non-empty and <= {max_len} bytes"))
    }
}

fn require_tagged_hash(label: &str, value: &str, prefix: &str) -> Result<(), String> {
    let digest = value
        .strip_prefix(prefix)
        .ok_or_else(|| format!("{label} must use {prefix}<hex>"))?;
    if digest.len() != 64
        || !digest
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
    {
        return Err(format!(
            "{label} must contain exactly 64 lowercase hexadecimal digits"
        ));
    }
    Ok(())
}

fn push_str(hasher: &mut blake3::Hasher, value: &str) {
    let bytes = value.as_bytes();
    hasher.update(&(bytes.len() as u64).to_be_bytes());
    hasher.update(bytes);
}

fn tagged(prefix: &str, hash: blake3::Hash) -> String {
    format!("{prefix}{}", hash.to_hex())
}

fn derive_provider_idempotency_key_inner(
    native_replay_identity: &str,
    action_key_digest: &str,
    provider_environment: &str,
    provider_audience: &str,
    adapter_identity: &str,
) -> String {
    let mut hasher = blake3::Hasher::new();
    hasher.update(PROVIDER_IDEMPOTENCY_KEY_DOMAIN);
    push_str(&mut hasher, native_replay_identity);
    push_str(&mut hasher, action_key_digest);
    push_str(&mut hasher, provider_environment);
    push_str(&mut hasher, provider_audience);
    push_str(&mut hasher, adapter_identity);
    format!("{}{}", PROVIDER_IDEMPOTENCY_KEY_PREFIX, hasher.finalize().to_hex())
}

pub fn derive_provider_idempotency_key(
    native_replay_identity: &str,
    action_key_digest: &str,
    provider_environment: &str,
    provider_audience: &str,
    adapter_identity: &str,
) -> Result<String, String> {
    require_opaque("native_replay_identity", native_replay_identity, MAX_REF_LEN)?;
    require_tagged_hash("action_key_digest", action_key_digest, crate::ACTION_KEY_PREFIX)?;
    require_opaque("provider_environment", provider_environment, MAX_REF_LEN)?;
    require_opaque("provider_audience", provider_audience, MAX_REF_LEN)?;
    require_opaque("adapter_identity", adapter_identity, MAX_REF_LEN)?;
    Ok(derive_provider_idempotency_key_inner(
        native_replay_identity,
        action_key_digest,
        provider_environment,
        provider_audience,
        adapter_identity,
    ))
}

fn reconciliation_token(owner_token: &str, attempt_identity: &str, action_key_digest: &str) -> String {
    let mut hasher = blake3::Hasher::new();
    hasher.update(b"MYCELIX-CONSTITUTIONAL-RECONCILIATION-TOKEN\0V1\0");
    push_str(&mut hasher, owner_token);
    push_str(&mut hasher, attempt_identity);
    push_str(&mut hasher, action_key_digest);
    tagged("constitutional-reconciliation-token-v1:", hasher.finalize())
}


fn attempt_state_tag(state: AttemptRecordState) -> u8 {
    match state {
        AttemptRecordState::Consumed => 1,
        AttemptRecordState::Reserved => 2,
        AttemptRecordState::DispatchPending => 3,
        AttemptRecordState::Invoked => 4,
        AttemptRecordState::Executed => 5,
        AttemptRecordState::Failed => 6,
        AttemptRecordState::Indeterminate => 7,
        AttemptRecordState::NotEntered => 8,
    }
}

fn fence_state_tag(state: ActionFenceState) -> u8 {
    match state {
        ActionFenceState::Occupied => 1,
        ActionFenceState::Closed => 2,
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TerminalOutcomeV1 {
    Executed,
    Failed,
}

fn terminal_outcome_tag(outcome: TerminalOutcomeV1) -> u8 {
    match outcome {
        TerminalOutcomeV1::Executed => 1,
        TerminalOutcomeV1::Failed => 2,
    }
}

/// Identity of independently verified terminal outcome evidence.
///
/// The attempt owner and the outcome verifier are intentionally separate
/// authorities. This type binds the verifier's evidence to the exact same-action
/// key and concrete attempt being transitioned.
#[derive(Debug, Clone, PartialEq, Eq, Hash)]
pub struct TerminalEvidenceV1 {
    action_key_digest: String,
    attempt_identity: String,
    operation_id: String,
    native_replay_identity: String,
    effecting_target_identity: String,
    provider_environment: String,
    provider_audience: String,
    adapter_identity: String,
    provider_idempotency_key: String,
    outcome: TerminalOutcomeV1,
    evidence_commitment: String,
    verifier_identity: String,
    digest: String,
}

impl TerminalEvidenceV1 {
    pub fn from_attempt(
        action_key: &ActionKeyV1,
        attempt: &AttemptRecordV1,
        outcome: TerminalOutcomeV1,
        provider_idempotency_key: impl Into<String>,
        evidence_commitment: impl Into<String>,
        verifier_identity: impl Into<String>,
    ) -> Result<Self, String> {
        let provider_idempotency_key = provider_idempotency_key.into();
        let evidence_commitment = evidence_commitment.into();
        let verifier_identity = verifier_identity.into();
        require_opaque(
            "provider_idempotency_key",
            &provider_idempotency_key,
            MAX_REF_LEN,
        )?;
        require_tagged_hash("action_key_digest", action_key.digest(), crate::ACTION_KEY_PREFIX)?;
        require_tagged_hash("attempt_identity", &attempt.attempt_identity, crate::ATTEMPT_IDENTITY_PREFIX)?;
        attempt.validate()?;
        if attempt.action_key_digest != action_key.digest() {
            return Err("terminal evidence action key does not match attempt".into());
        }
        require_opaque("evidence_commitment", &evidence_commitment, MAX_REF_LEN)?;
        require_opaque("verifier_identity", &verifier_identity, MAX_REF_LEN)?;

        let mut hasher = blake3::Hasher::new();
        hasher.update(b"MYCELIX-CONSTITUTIONAL-TERMINAL-EVIDENCE\0V3\0");
        hasher.update(&ATTEMPT_RECORD_SCHEMA_VERSION.to_be_bytes());
        push_str(&mut hasher, action_key.digest());
        push_str(&mut hasher, &attempt.attempt_identity);
        push_str(&mut hasher, &attempt.operation_id);
        push_str(&mut hasher, &attempt.native_replay_identity);
        push_str(&mut hasher, &attempt.effecting_target_identity);
        push_str(&mut hasher, &attempt.provider_environment);
        push_str(&mut hasher, &attempt.provider_audience);
        push_str(&mut hasher, &attempt.adapter_identity);
        push_str(&mut hasher, &provider_idempotency_key);
        hasher.update(&[terminal_outcome_tag(outcome)]);
        push_str(&mut hasher, &evidence_commitment);
        push_str(&mut hasher, &verifier_identity);

        Ok(Self {
            action_key_digest: action_key.digest().to_owned(),
            attempt_identity: attempt.attempt_identity.clone(),
            operation_id: attempt.operation_id.clone(),
            native_replay_identity: attempt.native_replay_identity.clone(),
            effecting_target_identity: attempt.effecting_target_identity.clone(),
            provider_environment: attempt.provider_environment.clone(),
            provider_audience: attempt.provider_audience.clone(),
            adapter_identity: attempt.adapter_identity.clone(),
            provider_idempotency_key,
            outcome,
            evidence_commitment,
            verifier_identity,
            digest: tagged("constitutional-terminal-evidence-v3:", hasher.finalize()),
        })
    }

    pub fn action_key_digest(&self) -> &str {
        &self.action_key_digest
    }

    pub fn attempt_identity(&self) -> &str {
        &self.attempt_identity
    }

    pub fn outcome(&self) -> TerminalOutcomeV1 {
        self.outcome
    }
    pub fn operation_id(&self) -> &str { &self.operation_id }
    pub fn native_replay_identity(&self) -> &str { &self.native_replay_identity }
    pub fn effecting_target_identity(&self) -> &str { &self.effecting_target_identity }
    pub fn provider_environment(&self) -> &str { &self.provider_environment }
    pub fn provider_audience(&self) -> &str { &self.provider_audience }
    pub fn adapter_identity(&self) -> &str { &self.adapter_identity }
    pub fn provider_idempotency_key(&self) -> &str { &self.provider_idempotency_key }


    pub fn evidence_commitment(&self) -> &str {
        &self.evidence_commitment
    }

    pub fn verifier_identity(&self) -> &str {
        &self.verifier_identity
    }

    /// The verifier identity participates in this digest as provenance. Its
    /// authenticity/authorization is a deployment concern outside this pure type.
    pub fn digest(&self) -> &str {
        &self.digest
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum AttemptRecordState {
    Consumed,
    Reserved,
    DispatchPending,
    Invoked,
    Executed,
    Failed,
    Indeterminate,
    NotEntered,
}

impl AttemptRecordState {
    /// Stable integer encoding for durable stores.
    ///
    /// This encoding is part of the v1 persistence contract; callers must not
    /// invent a second state mapping in each storage adapter.
    pub fn storage_tag(self) -> i64 {
        attempt_state_tag(self) as i64
    }

    /// Decode the stable v1 durable-store state encoding.
    pub fn from_storage_tag(tag: i64) -> Option<Self> {
        match tag {
            1 => Some(Self::Consumed),
            2 => Some(Self::Reserved),
            3 => Some(Self::DispatchPending),
            4 => Some(Self::Invoked),
            5 => Some(Self::Executed),
            6 => Some(Self::Failed),
            7 => Some(Self::Indeterminate),
            8 => Some(Self::NotEntered),
            _ => None,
        }
    }

    pub fn occupies_action_fence(self) -> bool {
        matches!(
            self,
            Self::Consumed
                | Self::Reserved
                | Self::DispatchPending
                | Self::Invoked
                | Self::Indeterminate
        )
    }

    pub fn is_terminal(self) -> bool {
        matches!(self, Self::Executed | Self::Failed | Self::NotEntered)
    }

    pub fn allows_transition_to(self, next: Self) -> bool {
        matches!(
            (self, next),
            (Self::Consumed, Self::DispatchPending)
                | (Self::Reserved, Self::DispatchPending)
                | (Self::DispatchPending, Self::Invoked)
                | (Self::DispatchPending, Self::Indeterminate)
                | (Self::Invoked, Self::Indeterminate)
        )
    }

    pub fn allows_not_entered_release(self) -> bool {
        matches!(self, Self::Consumed | Self::Reserved)
    }
}

/// Exact authorization decision accepted by the effect boundary before
/// durable consumption, reservation, or provider entry.
///
/// This is an evidence object for the authorization decision, not a new
/// authorization mechanism. Its scope is the exact observed attempt and action;
/// the verifier identity and point-in-time snapshots are preserved by the digest.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorizationAdmissionProofV1 {
    attempt_identity: String,
    action_key_digest: String,
    operation_id: String,
    native_replay_identity: String,
    action_digest: String,
    effecting_target_identity: String,
    provider_environment: String,
    provider_audience: String,
    adapter_identity: String,
    authorization_snapshot_digest: String,
    policy_snapshot_digest: String,
    status_snapshot_digest: String,
    checked_at_unix_ms: u64,
    valid_until_unix_ms: u64,
    verifier_identity: String,
    digest: String,
}

impl AuthorizationAdmissionProofV1 {
    pub fn new(
        attempt: &AttemptRecordV1,
        action_key: &ActionKeyV1,
        checked_at_unix_ms: u64,
        valid_until_unix_ms: u64,
        authorization_snapshot_digest: impl Into<String>,
        policy_snapshot_digest: impl Into<String>,
        status_snapshot_digest: impl Into<String>,
        verifier_identity: impl Into<String>,
    ) -> Result<Self, String> {
        let authorization_snapshot_digest = authorization_snapshot_digest.into();
        let policy_snapshot_digest = policy_snapshot_digest.into();
        let status_snapshot_digest = status_snapshot_digest.into();
        let verifier_identity = verifier_identity.into();

        if valid_until_unix_ms <= checked_at_unix_ms {
            return Err("authorization admission proof validity window is empty".into());
        }
        if attempt.attempt_identity.is_empty()
            || attempt.action_key_digest != action_key.digest()
            || attempt.action_digest != action_key.material_action_digest()
            || attempt.effecting_target_identity != action_key.effecting_target_identity()
        {
            return Err("authorization admission proof scope does not match observed action".into());
        }
        for (label, value) in [
            ("authorization_snapshot_digest", authorization_snapshot_digest.as_str()),
            ("policy_snapshot_digest", policy_snapshot_digest.as_str()),
            ("status_snapshot_digest", status_snapshot_digest.as_str()),
            ("verifier_identity", verifier_identity.as_str()),
        ] {
            require_opaque(label, value, MAX_REF_LEN)?;
        }

        let mut hasher = blake3::Hasher::new();
        hasher.update(b"MYCELIX-CONSTITUTIONAL-AUTHORIZATION-ADMISSION-PROOF\0V1\0");
        push_str(&mut hasher, &attempt.attempt_identity);
        push_str(&mut hasher, action_key.digest());
        push_str(&mut hasher, &attempt.operation_id);
        push_str(&mut hasher, &attempt.native_replay_identity);
        push_str(&mut hasher, &attempt.action_digest);
        push_str(&mut hasher, &attempt.effecting_target_identity);
        push_str(&mut hasher, &attempt.provider_environment);
        push_str(&mut hasher, &attempt.provider_audience);
        push_str(&mut hasher, &attempt.adapter_identity);
        push_str(&mut hasher, &authorization_snapshot_digest);
        push_str(&mut hasher, &policy_snapshot_digest);
        push_str(&mut hasher, &status_snapshot_digest);
        hasher.update(&checked_at_unix_ms.to_be_bytes());
        hasher.update(&valid_until_unix_ms.to_be_bytes());
        push_str(&mut hasher, &verifier_identity);

        Ok(Self {
            attempt_identity: attempt.attempt_identity.clone(),
            action_key_digest: action_key.digest().to_owned(),
            operation_id: attempt.operation_id.clone(),
            native_replay_identity: attempt.native_replay_identity.clone(),
            action_digest: attempt.action_digest.clone(),
            effecting_target_identity: attempt.effecting_target_identity.clone(),
            provider_environment: attempt.provider_environment.clone(),
            provider_audience: attempt.provider_audience.clone(),
            adapter_identity: attempt.adapter_identity.clone(),
            authorization_snapshot_digest,
            policy_snapshot_digest,
            status_snapshot_digest,
            checked_at_unix_ms,
            valid_until_unix_ms,
            verifier_identity,
            digest: format!(
                "constitutional-authorization-admission-proof-v1:{}",
                hasher.finalize().to_hex()
            ),
        })
    }

    pub fn matches(&self, attempt: &AttemptRecordV1, action_key: &ActionKeyV1) -> bool {
        self.attempt_identity == attempt.attempt_identity
            && self.action_key_digest == action_key.digest()
            && self.operation_id == attempt.operation_id
            && self.native_replay_identity == attempt.native_replay_identity
            && self.action_digest == attempt.action_digest
            && self.effecting_target_identity == attempt.effecting_target_identity
            && self.provider_environment == attempt.provider_environment
            && self.provider_audience == attempt.provider_audience
            && self.adapter_identity == attempt.adapter_identity
    }

    pub fn is_fresh(&self, now_unix_ms: u64) -> bool {
        self.checked_at_unix_ms <= now_unix_ms && now_unix_ms < self.valid_until_unix_ms
    }

    pub fn digest(&self) -> &str {
        &self.digest
    }

    pub fn verifier_identity(&self) -> &str {
        &self.verifier_identity
    }
}

/// Durable attempt record projection.
///
/// Identity-bearing roots are stored as their explicit digests/identifiers rather
/// than as serialized ActionKeyV1 / AttemptIdentityV1 values. The identity types
/// themselves remain non-serializable and private-fielded.
///
/// The provider idempotency key is persisted as part of this projection so the
/// downstream identity cannot silently change across restart or binary upgrade.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AttemptRecordV1 {
    pub schema_version: u16,
    pub attempt_identity: String,
    pub operation_id: String,
    pub native_replay_identity: String,
    pub action_digest: String,
    pub action_key_digest: String,
    pub effecting_target_identity: String,
    pub provider_reference_seed_digest: Option<String>,
    pub provider_reference_descriptor_digest: Option<String>,
    pub provider_environment: String,
    pub provider_audience: String,
    pub adapter_identity: String,
    pub provider_idempotency_key: String,
    pub authorization_admission_proof_digest: Option<String>,
    pub entry_admission_proof_digest: Option<String>,
    pub ownership_token_digest: String,
    pub reconciliation_token_digest: Option<String>,
    pub terminal_evidence_digest: Option<String>,
    pub state: AttemptRecordState,
    pub not_entered_marker: Option<String>,
}

impl AttemptRecordV1 {
    pub fn new(
        attempt_identity: &AttemptIdentityV1,
        operation_id: impl Into<String>,
        native_replay_identity: impl Into<String>,
        action_digest: impl Into<String>,
        action_key: &ActionKeyV1,
        provider_reference_seed_digest: Option<String>,
        provider_reference_descriptor_digest: Option<String>,
        provider_environment: impl Into<String>,
        provider_audience: impl Into<String>,
        adapter_identity: impl Into<String>,
        ownership_token_digest: impl Into<String>,
        state: AttemptRecordState,
    ) -> Result<Self, String> {
        let operation_id = operation_id.into();
        let native_replay_identity = native_replay_identity.into();
        let action_digest = action_digest.into();
        let provider_environment = provider_environment.into();
        let provider_audience = provider_audience.into();
        let adapter_identity = adapter_identity.into();
        let ownership_token_digest = ownership_token_digest.into();

        require_tagged_hash("attempt_identity", attempt_identity.digest(), crate::ATTEMPT_IDENTITY_PREFIX)?;
        require_tagged_hash("action_key_digest", action_key.digest(), crate::ACTION_KEY_PREFIX)?;
        require_opaque("operation_id", &operation_id, MAX_ID_LEN)?;
        require_opaque(
            "native_replay_identity",
            &native_replay_identity,
            MAX_REF_LEN,
        )?;
        require_opaque("action_digest", &action_digest, MAX_REF_LEN)?;
        require_opaque(
            "effecting_target_identity",
            action_key.effecting_target_identity(),
            MAX_REF_LEN,
        )?;
        require_opaque("provider_environment", &provider_environment, MAX_REF_LEN)?;
        require_opaque("provider_audience", &provider_audience, MAX_REF_LEN)?;
        require_opaque("adapter_identity", &adapter_identity, MAX_REF_LEN)?;
        let provider_idempotency_key = derive_provider_idempotency_key(
            &native_replay_identity,
            action_key.digest(),
            &provider_environment,
            &provider_audience,
            &adapter_identity,
        )?;
        require_opaque("ownership_token_digest", &ownership_token_digest, MAX_REF_LEN)?;

        if action_digest != action_key.material_action_digest() {
            return Err("action_digest does not match ActionKeyV1 material action digest".into());
        }

        if let Some(value) = &provider_reference_seed_digest {
            require_opaque("provider_reference_seed_digest", value, MAX_REF_LEN)?;
        }
        if let Some(value) = &provider_reference_descriptor_digest {
            require_opaque(
                "provider_reference_descriptor_digest",
                value,
                MAX_REF_LEN,
            )?;
        }

        let out = Self {
            schema_version: ATTEMPT_RECORD_SCHEMA_VERSION,
            attempt_identity: attempt_identity.digest().to_owned(),
            operation_id,
            native_replay_identity,
            action_digest,
            action_key_digest: action_key.digest().to_owned(),
            effecting_target_identity: action_key.effecting_target_identity().to_owned(),
            provider_reference_seed_digest,
            provider_reference_descriptor_digest,
            provider_environment,
            provider_audience,
            adapter_identity,
            provider_idempotency_key,
            authorization_admission_proof_digest: None,
            entry_admission_proof_digest: None,
            ownership_token_digest,
            reconciliation_token_digest: None,
            terminal_evidence_digest: None,
            state,
            not_entered_marker: None,
        };
        out.validate()?;
        Ok(out)
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != ATTEMPT_RECORD_SCHEMA_VERSION {
            return Err("unsupported attempt record schema version".into());
        }
        require_tagged_hash(
            "attempt_identity",
            &self.attempt_identity,
            crate::ATTEMPT_IDENTITY_PREFIX,
        )?;
        require_tagged_hash(
            "action_key_digest",
            &self.action_key_digest,
            crate::ACTION_KEY_PREFIX,
        )?;
        require_opaque("operation_id", &self.operation_id, MAX_ID_LEN)?;
        require_opaque(
            "native_replay_identity",
            &self.native_replay_identity,
            MAX_REF_LEN,
        )?;
        require_opaque("action_digest", &self.action_digest, MAX_REF_LEN)?;
        require_opaque(
            "effecting_target_identity",
            &self.effecting_target_identity,
            MAX_REF_LEN,
        )?;
        require_opaque(
            "provider_environment",
            &self.provider_environment,
            MAX_REF_LEN,
        )?;
        require_opaque("provider_audience", &self.provider_audience, MAX_REF_LEN)?;
        require_opaque("adapter_identity", &self.adapter_identity, MAX_REF_LEN)?;
        if let Some(proof) = &self.authorization_admission_proof_digest {
            require_opaque("authorization_admission_proof_digest", proof, MAX_REF_LEN)?;
        }
        if let Some(proof) = &self.entry_admission_proof_digest {
            require_opaque("entry_admission_proof_digest", proof, MAX_REF_LEN)?;
        }
        if matches!(
            self.state,
            AttemptRecordState::Consumed
                | AttemptRecordState::Reserved
                | AttemptRecordState::NotEntered
        ) && self.entry_admission_proof_digest.is_some()
        {
            return Err(
                "entry_admission_proof_digest is invalid before provider-entry admission"
                    .into(),
            );
        }
        require_opaque(
            "provider_idempotency_key",
            &self.provider_idempotency_key,
            MAX_REF_LEN,
        )?;
        let expected_provider_idempotency_key = derive_provider_idempotency_key(
            &self.native_replay_identity,
            &self.action_key_digest,
            &self.provider_environment,
            &self.provider_audience,
            &self.adapter_identity,
        )?;
        if self.provider_idempotency_key != expected_provider_idempotency_key {
            return Err("provider_idempotency_key does not match versioned derivation".into());
        }
        require_opaque(
            "ownership_token_digest",
            &self.ownership_token_digest,
            MAX_REF_LEN,
        )?;

        if let Some(value) = &self.provider_reference_seed_digest {
            require_opaque("provider_reference_seed_digest", value, MAX_REF_LEN)?;
        }
        if let Some(value) = &self.provider_reference_descriptor_digest {
            require_opaque(
                "provider_reference_descriptor_digest",
                value,
                MAX_REF_LEN,
            )?;
        }

        if matches!(
            self.state,
            AttemptRecordState::DispatchPending
                | AttemptRecordState::Invoked
                | AttemptRecordState::Executed
                | AttemptRecordState::Failed
                | AttemptRecordState::Indeterminate
        ) && (self.provider_reference_seed_digest.is_none()
            || self.provider_reference_descriptor_digest.is_none())
        {
            return Err(
                "post-reservation attempt states require provider reference seed and descriptor digests"
                    .into(),
            );
        }

        if matches!(
            self.state,
            AttemptRecordState::Executed | AttemptRecordState::Failed
        ) && self.terminal_evidence_digest.is_none()
        {
            return Err("terminal outcome requires terminal_evidence_digest".into());
        }
        if !matches!(
            self.state,
            AttemptRecordState::Executed | AttemptRecordState::Failed
        ) && self.terminal_evidence_digest.is_some()
        {
            return Err(
                "terminal_evidence_digest is only valid for terminal outcomes".into(),
            );
        }
        if let Some(token) = &self.reconciliation_token_digest {
            require_opaque("reconciliation_token_digest", token, MAX_REF_LEN)?;
        }
        if self.state == AttemptRecordState::Indeterminate && self.reconciliation_token_digest.is_none() {
            return Err("Indeterminate requires reconciliation_token_digest".into());
        }
        if self.state != AttemptRecordState::Indeterminate && self.reconciliation_token_digest.is_some() {
            return Err("reconciliation_token_digest is only valid for Indeterminate".into());
        }

        if let Some(evidence) = &self.terminal_evidence_digest {
            require_opaque("terminal_evidence_digest", evidence, MAX_REF_LEN)?;
        }

        if self.state == AttemptRecordState::NotEntered && self.not_entered_marker.is_none() {
            return Err("NotEntered requires an explicit not_entered_marker".into());
        }
        if self.state != AttemptRecordState::NotEntered && self.not_entered_marker.is_some() {
            return Err("not_entered_marker is only valid for NotEntered".into());
        }
        if let Some(marker) = &self.not_entered_marker {
            require_opaque("not_entered_marker", marker, MAX_REF_LEN)?;
        }

        Ok(())
    }

    /// Digest of the complete durable projection. This is record integrity, not
    /// semantic action identity: `operation_id` is intentionally included here.
    pub fn provider_idempotency_key(&self) -> &str {
        &self.provider_idempotency_key
    }

    pub fn authorization_admission_proof_digest(&self) -> Option<&str> {
        self.authorization_admission_proof_digest.as_deref()
    }

    pub fn entry_admission_proof_digest(&self) -> Option<&str> {
        self.entry_admission_proof_digest.as_deref()
    }

    pub fn reconciliation_token_digest(&self) -> String {
        reconciliation_token(
            &self.ownership_token_digest,
            &self.attempt_identity,
            &self.action_key_digest,
        )
    }

    pub fn record_digest(&self) -> String {
        let mut hasher = blake3::Hasher::new();
        hasher.update(ATTEMPT_RECORD_DOMAIN);
        hasher.update(&self.schema_version.to_be_bytes());
        push_str(&mut hasher, &self.attempt_identity);
        push_str(&mut hasher, &self.operation_id);
        push_str(&mut hasher, &self.native_replay_identity);
        push_str(&mut hasher, &self.action_digest);
        push_str(&mut hasher, &self.action_key_digest);
        push_str(&mut hasher, &self.effecting_target_identity);
        push_str(
            &mut hasher,
            self.provider_reference_seed_digest.as_deref().unwrap_or(""),
        );
        push_str(
            &mut hasher,
            self.provider_reference_descriptor_digest
                .as_deref()
                .unwrap_or(""),
        );
        push_str(&mut hasher, &self.provider_environment);
        push_str(&mut hasher, &self.provider_audience);
        push_str(&mut hasher, &self.adapter_identity);
        push_str(
            &mut hasher,
            self.entry_admission_proof_digest.as_deref().unwrap_or(""),
        );
        push_str(&mut hasher, &self.provider_idempotency_key);
        push_str(
            &mut hasher,
            self.authorization_admission_proof_digest
                .as_deref()
                .unwrap_or(""),
        );
        push_str(&mut hasher, &self.ownership_token_digest);
        push_str(
            &mut hasher,
            self.reconciliation_token_digest.as_deref().unwrap_or(""),
        );
        push_str(
            &mut hasher,
            self.terminal_evidence_digest.as_deref().unwrap_or(""),
        );
        hasher.update(&[attempt_state_tag(self.state)]);
        push_str(&mut hasher, self.not_entered_marker.as_deref().unwrap_or(""));
        tagged(ATTEMPT_RECORD_PREFIX, hasher.finalize())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ActionFenceState {
    Occupied,
    Closed,
}

impl ActionFenceState {
    /// Stable integer encoding for durable stores.
    pub fn storage_tag(self) -> i64 {
        fence_state_tag(self) as i64
    }

    /// Decode the stable v1 durable-store state encoding.
    pub fn from_storage_tag(tag: i64) -> Option<Self> {
        match tag {
            1 => Some(Self::Occupied),
            2 => Some(Self::Closed),
            _ => None,
        }
    }

    pub fn is_closed(self) -> bool {
        matches!(self, Self::Closed)
    }
}

/// Durable same-action fence row, keyed only by ActionKeyV1's digest.
///
/// Operation identifiers, native replay identities, and attempt identifiers are
/// owner metadata, never the collision namespace.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ActionFenceRecordV1 {
    pub schema_version: u16,
    pub action_key_digest: String,
    pub owner_attempt_identity: String,
    pub owner_token_digest: String,
    pub state: ActionFenceState,
}

impl ActionFenceRecordV1 {
    fn new(
        action_key: &ActionKeyV1,
        owner_attempt_identity: &AttemptIdentityV1,
        owner_token_digest: impl Into<String>,
        state: ActionFenceState,
    ) -> Result<Self, String> {
        let owner_token_digest = owner_token_digest.into();
        require_opaque("owner_token_digest", &owner_token_digest, MAX_REF_LEN)?;
        let out = Self {
            schema_version: ACTION_FENCE_SCHEMA_VERSION,
            action_key_digest: action_key.digest().to_owned(),
            owner_attempt_identity: owner_attempt_identity.digest().to_owned(),
            owner_token_digest,
            state,
        };
        out.validate()?;
        Ok(out)
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != ACTION_FENCE_SCHEMA_VERSION {
            return Err("unsupported action fence schema version".into());
        }
        require_tagged_hash(
            "action_key_digest",
            &self.action_key_digest,
            crate::ACTION_KEY_PREFIX,
        )?;
        require_tagged_hash(
            "owner_attempt_identity",
            &self.owner_attempt_identity,
            crate::ATTEMPT_IDENTITY_PREFIX,
        )?;
        require_opaque("owner_token_digest", &self.owner_token_digest, MAX_REF_LEN)?;
        Ok(())
    }

    pub fn record_digest(&self) -> String {
        let mut hasher = blake3::Hasher::new();
        hasher.update(ACTION_FENCE_RECORD_DOMAIN);
        hasher.update(&self.schema_version.to_be_bytes());
        push_str(&mut hasher, &self.action_key_digest);
        push_str(&mut hasher, &self.owner_attempt_identity);
        push_str(&mut hasher, &self.owner_token_digest);
        hasher.update(&[fence_state_tag(self.state)]);
        tagged(ACTION_FENCE_RECORD_PREFIX, hasher.finalize())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AtomicAdmissionDecision {
    Admitted,
    DuplicateAttempt,
    ActionInFlight,
    ActionAlreadyExecuted,
    NativeReplayConflict {
        existing_operation_id: String,
        existing_action_key_digest: String,
    },
    AttemptOwnershipConflict,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ActionFenceMutationError {
    NotOwner,
    OwnershipTokenMismatch,
    NotOccupied,
    AlreadyClosed,
    InvalidTransition,
    TerminalEvidenceMismatch,
    ProviderEntryClaimed,
    ProviderEntryProofAlreadyRecorded,
    ProviderEntryClaimMismatch,
    /// The durable storage layer failed. This is deliberately distinct from a
    /// semantic denial so callers cannot reinterpret infrastructure failure as a
    /// successful release/transition.
    StorageFailure(String),
}

/// Durable-store contract for the same-action fence.
///
/// An implementation of this trait is a deployment boundary, not a proof that
/// merely holding a Rust value is durable. The backing store MUST make each
/// method below a conflict-detecting atomic transition in the same durable,
/// shared state domain used for native replay consumption/reservation.
///
/// In particular, atomically_admit MUST make the attempt record and same-action
/// fence visible as one linearized admission decision to all boundary instances
/// that can reach the effecting target.
pub trait DurableActionFenceStore {
    fn atomically_admit(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        record: AttemptRecordV1,
        authorization_proof: AuthorizationAdmissionProofV1,
    ) -> Result<AtomicAdmissionDecision, String>;

    /// Advance a durable attempt through the non-terminal lifecycle. Each method
    /// MUST be a single conflict-detecting durable transaction.
    fn atomically_mark_dispatch_pending(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
    ) -> Result<(), ActionFenceMutationError>;

    /// Establish a durable single-winner claim between DISPATCH_PENDING and
    /// provider entry. A live claim blocks reconciliation and every other
    /// provider-entry claimant for this exact attempt.
    fn atomically_claim_provider_entry(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        claim_token_digest: &str,
    ) -> Result<ProviderEntryClaimV1, ActionFenceMutationError>;

    /// Complete the provider-entry claim and durably record INVOKED in the
    /// same transaction. The claim itself is consumed and cannot be replayed.
    fn atomically_mark_invoked(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        claim_token_digest: &str,
    ) -> Result<(), ActionFenceMutationError>;

    /// Persist the exact final-entry proof digest while the provider-entry claim
    /// remains held.
    fn atomically_record_provider_entry_proof(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        proof_digest: String,
    ) -> Result<(), ActionFenceMutationError>;

    /// Atomically release a still-held provider-entry claim as NotEntered.
    fn atomically_release_provider_entry_claim_not_entered(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        claim_token_digest: &str,
        marker: String,
    ) -> Result<(), ActionFenceMutationError>;

    /// Explicitly abandon a stranded provider-entry claim and atomically move
    /// the attempt to INDETERMINATE. This is a recovery operation, not a lease
    /// timeout: callers must supply externally authenticated recovery authority.
    fn atomically_recover_provider_entry_claim(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        claim_token_digest: &str,
    ) -> Result<String, ActionFenceMutationError>;

    fn atomically_mark_indeterminate(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
    ) -> Result<String, ActionFenceMutationError>;

    fn atomically_release_after_failed(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        terminal_evidence: &TerminalEvidenceV1,
    ) -> Result<(), ActionFenceMutationError>;

    fn atomically_close_executed(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        terminal_evidence: &TerminalEvidenceV1,
    ) -> Result<(), ActionFenceMutationError>;

    fn atomically_release_not_entered(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        marker: String,
    ) -> Result<(), ActionFenceMutationError>;

    /// Durable reads are part of the contract: a caller must be able to prove
    /// the persisted state it is about to act on after restart, rather than
    /// relying on an in-memory cache or a DHT discovery index.
    fn durably_read_attempt(
        &self,
        attempt_identity: &AttemptIdentityV1,
    ) -> Result<Option<AttemptRecordV1>, String>;

    fn durably_read_fence(
        &self,
        action_key: &ActionKeyV1,
    ) -> Result<Option<ActionFenceRecordV1>, String>;

    fn durably_read_replay_binding(
        &self,
        native_replay_identity: &str,
    ) -> Result<Option<NativeReplayBindingV1>, String>;

    fn durably_read_provider_entry_claim(
        &self,
        attempt_identity: &AttemptIdentityV1,
    ) -> Result<Option<ProviderEntryClaimV1>, String>;
}

/// Durable binding for one native replay identity.
///
/// The binding is intentionally separate from ActionKeyV1 and AttemptIdentityV1.
/// A replay identity may span multiple retries of the same authorized operation,
/// but may not be silently rebound to another operation or material action.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct NativeReplayBindingV1 {
    pub schema_version: u16,
    pub native_replay_identity: String,
    pub operation_id: String,
    pub action_key_digest: String,
}

impl NativeReplayBindingV1 {
    fn new(
        native_replay_identity: impl Into<String>,
        operation_id: impl Into<String>,
        action_key_digest: impl Into<String>,
    ) -> Result<Self, String> {
        let out = Self {
            schema_version: NATIVE_REPLAY_BINDING_SCHEMA_VERSION,
            native_replay_identity: native_replay_identity.into(),
            operation_id: operation_id.into(),
            action_key_digest: action_key_digest.into(),
        };
        out.validate()?;
        Ok(out)
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != NATIVE_REPLAY_BINDING_SCHEMA_VERSION {
            return Err("unsupported native replay binding schema version".into());
        }
        require_opaque(
            "native_replay_identity",
            &self.native_replay_identity,
            MAX_REF_LEN,
        )?;
        require_opaque("operation_id", &self.operation_id, MAX_ID_LEN)?;
        require_tagged_hash(
            "action_key_digest",
            &self.action_key_digest,
            crate::ACTION_KEY_PREFIX,
        )?;
        Ok(())
    }

    pub fn digest(&self) -> String {
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"MYCELIX-CONSTITUTIONAL-NATIVE-REPLAY-BINDING\0V1\0");
        hasher.update(&self.schema_version.to_be_bytes());
        push_str(&mut hasher, &self.native_replay_identity);
        push_str(&mut hasher, &self.operation_id);
        push_str(&mut hasher, &self.action_key_digest);
        tagged(NATIVE_REPLAY_BINDING_PREFIX, hasher.finalize())
    }
}

/// Durable coordination claim that serializes the final step before
/// provider entry. It is deliberately separate from the attempt lifecycle state:
/// a claim can be held while the provider call is in progress without relabeling
/// the observation as INVOKED before the call has actually begun.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProviderEntryClaimV1 {
    pub schema_version: u16,
    pub attempt_identity: String,
    pub action_key_digest: String,
    pub owner_token_digest: String,
    pub claim_token_digest: String,
    record_digest: String,
}

impl ProviderEntryClaimV1 {
    pub fn new(
        attempt_identity: impl Into<String>,
        action_key_digest: impl Into<String>,
        owner_token_digest: impl Into<String>,
        claim_token_digest: impl Into<String>,
    ) -> Result<Self, String> {
        let out = Self {
            schema_version: 1,
            attempt_identity: attempt_identity.into(),
            action_key_digest: action_key_digest.into(),
            owner_token_digest: owner_token_digest.into(),
            claim_token_digest: claim_token_digest.into(),
            record_digest: String::new(),
        };
        require_tagged_hash("attempt_identity", &out.attempt_identity, crate::ATTEMPT_IDENTITY_PREFIX)?;
        require_tagged_hash("action_key_digest", &out.action_key_digest, crate::ACTION_KEY_PREFIX)?;
        require_opaque("owner_token_digest", &out.owner_token_digest, MAX_REF_LEN)?;
        require_opaque("claim_token_digest", &out.claim_token_digest, MAX_REF_LEN)?;
        let mut out = out;
        out.record_digest = out.compute_digest();
        Ok(out)
    }

    fn compute_digest(&self) -> String {
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"MYCELIX-CONSTITUTIONAL-PROVIDER-ENTRY-CLAIM\0V1\0");
        hasher.update(&self.schema_version.to_be_bytes());
        push_str(&mut hasher, &self.attempt_identity);
        push_str(&mut hasher, &self.action_key_digest);
        push_str(&mut hasher, &self.owner_token_digest);
        push_str(&mut hasher, &self.claim_token_digest);
        tagged("constitutional-provider-entry-claim-v1:", hasher.finalize())
    }

    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != 1 {
            return Err("unsupported provider entry claim schema version".into());
        }
        require_tagged_hash("attempt_identity", &self.attempt_identity, crate::ATTEMPT_IDENTITY_PREFIX)?;
        require_tagged_hash("action_key_digest", &self.action_key_digest, crate::ACTION_KEY_PREFIX)?;
        require_opaque("owner_token_digest", &self.owner_token_digest, MAX_REF_LEN)?;
        require_opaque("claim_token_digest", &self.claim_token_digest, MAX_REF_LEN)?;
        if self.record_digest != self.compute_digest() {
            return Err("provider entry claim digest mismatch".into());
        }
        Ok(())
    }

    pub fn record_digest(&self) -> &str {
        &self.record_digest
    }
}

/// Reference model for the required atomic admission transition.
///
/// The method `admit` is deliberately one mutation over the combined durable
/// state model: either the replay binding, attempt record, and action fence are
/// all installed, or neither changes. This demonstrates the conflict theorem
/// without claiming that the in-memory map itself provides crash durability.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct AtomicActionFenceModelV1 {
    attempts: BTreeMap<String, AttemptRecordV1>,
    fences: BTreeMap<String, ActionFenceRecordV1>,
    replay_bindings: BTreeMap<String, NativeReplayBindingV1>,
    provider_entry_claims: BTreeMap<String, ProviderEntryClaimV1>,
}

impl AtomicActionFenceModelV1 {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn admit(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        mut record: AttemptRecordV1,
        authorization_proof: AuthorizationAdmissionProofV1,
    ) -> Result<AtomicAdmissionDecision, String> {
        self.validate_invariants()?;
        if record.authorization_admission_proof_digest.is_some() {
            return Err(
                "attempt record must not supply its own authorization admission proof digest"
                    .into(),
            );
        }
        if !authorization_proof.matches(&record, action_key) {
            return Err("authorization admission proof does not match exact attempt/action".into());
        }
        record.authorization_admission_proof_digest = Some(authorization_proof.digest().to_owned());
        record.validate()?;

        if record.action_key_digest != action_key.digest() {
            return Err("attempt record action key does not match typed ActionKeyV1".into());
        }
        if record.attempt_identity != attempt_identity.digest() {
            return Err("attempt record identity does not match typed AttemptIdentityV1".into());
        }
        if record.action_digest != action_key.material_action_digest() {
            return Err("attempt record action digest does not match typed ActionKeyV1".into());
        }
        if record.effecting_target_identity != action_key.effecting_target_identity() {
            return Err(
                "attempt record effecting target does not match typed ActionKeyV1".into(),
            );
        }

        if let Some(existing) = self.attempts.get(&record.attempt_identity) {
            if existing.record_digest() == record.record_digest() {
                if existing.state.occupies_action_fence() {
                    match self.fences.get(&record.action_key_digest) {
                        Some(fence)
                            if fence.state == ActionFenceState::Occupied
                                && fence.owner_attempt_identity == record.attempt_identity
                                && fence.owner_token_digest
                                    == record.ownership_token_digest => {}
                        _ => return Ok(AtomicAdmissionDecision::AttemptOwnershipConflict),
                    }
                }
                return Ok(AtomicAdmissionDecision::DuplicateAttempt);
            }
            return Ok(AtomicAdmissionDecision::AttemptOwnershipConflict);
        }

        match self.replay_bindings.get(&record.native_replay_identity) {
            None => {}
            Some(existing)
                if existing.operation_id == record.operation_id
                    && existing.action_key_digest == record.action_key_digest => {}
            Some(existing) => {
                return Ok(AtomicAdmissionDecision::NativeReplayConflict {
                    existing_operation_id: existing.operation_id.clone(),
                    existing_action_key_digest: existing.action_key_digest.clone(),
                });
            }
        }

        match self.fences.get(record.action_key_digest.as_str()) {
            None => {}
            Some(existing) if existing.state.is_closed() => {
                return Ok(AtomicAdmissionDecision::ActionAlreadyExecuted);
            }
            Some(existing) if existing.owner_attempt_identity != record.attempt_identity => {
                return Ok(AtomicAdmissionDecision::ActionInFlight);
            }
            Some(_) => {
                return Ok(AtomicAdmissionDecision::AttemptOwnershipConflict);
            }
        }

        if !matches!(
            record.state,
            AttemptRecordState::Consumed | AttemptRecordState::Reserved
        ) {
            return Err(
                "admitted attempt must begin in Consumed or Reserved before DISPATCH_PENDING"
                    .into(),
            );
        }

        let fence = ActionFenceRecordV1::new(
            action_key,
            attempt_identity,
            record.ownership_token_digest.clone(),
            ActionFenceState::Occupied,
        )?;
        if fence.action_key_digest != record.action_key_digest
            || fence.owner_attempt_identity != record.attempt_identity
        {
            return Err("constructed action fence does not match attempt record".into());
        }

        let replay_binding = NativeReplayBindingV1::new(
            record.native_replay_identity.clone(),
            record.operation_id.clone(),
            record.action_key_digest.clone(),
        )?;

        // The replay binding, attempt record, and action fence are committed by this
        // one reference-model mutation. A durable implementation must map this to one
        // transaction/linearizable conflict-detecting write.
        self.replay_bindings
            .insert(replay_binding.native_replay_identity.clone(), replay_binding);
        self.attempts
            .insert(record.attempt_identity.clone(), record);
        self.fences.insert(fence.action_key_digest.clone(), fence);
        Ok(AtomicAdmissionDecision::Admitted)
    }

    pub fn fence(&self, action_key_digest: &str) -> Option<&ActionFenceRecordV1> {
        self.fences.get(action_key_digest)
    }

    pub fn attempt(&self, attempt_identity: &str) -> Option<&AttemptRecordV1> {
        self.attempts.get(attempt_identity)
    }

    pub fn replay_binding(&self, native_replay_identity: &str) -> Option<&NativeReplayBindingV1> {
        self.replay_bindings.get(native_replay_identity)
    }

    pub fn release_after_failed(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        terminal_evidence: &TerminalEvidenceV1,
    ) -> Result<(), ActionFenceMutationError> {
        self.transition_terminal(
            action_key,
            attempt_identity,
            owner_token_digest,
            terminal_evidence,
            AttemptRecordState::Failed,
        )
    }

    pub fn release_not_entered(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        marker: impl Into<String>,
    ) -> Result<(), ActionFenceMutationError> {
        let marker = marker.into();
        if !valid_opaque(&marker, MAX_REF_LEN) {
            return Err(ActionFenceMutationError::NotOccupied);
        }

        let current = self
            .attempts
            .get(attempt_identity.digest())
            .ok_or(ActionFenceMutationError::NotOwner)?;

        if current.ownership_token_digest != owner_token_digest {
            return Err(ActionFenceMutationError::OwnershipTokenMismatch);
        }
        if current.action_key_digest != action_key.digest() {
            return Err(ActionFenceMutationError::NotOwner);
        }
        if !current.state.allows_not_entered_release() {
            return Err(ActionFenceMutationError::NotOccupied);
        }

        let fence = self
            .fences
            .get(action_key.digest())
            .ok_or(ActionFenceMutationError::NotOccupied)?;
        if fence.owner_attempt_identity != attempt_identity.digest() {
            return Err(ActionFenceMutationError::NotOwner);
        }
        if fence.owner_token_digest != owner_token_digest {
            return Err(ActionFenceMutationError::OwnershipTokenMismatch);
        }
        if fence.state.is_closed() {
            return Err(ActionFenceMutationError::AlreadyClosed);
        }

        let attempt = self
            .attempts
            .get_mut(attempt_identity.digest())
            .ok_or(ActionFenceMutationError::NotOwner)?;
        attempt.state = AttemptRecordState::NotEntered;
        attempt.not_entered_marker = Some(marker);

        self.fences.remove(action_key.digest());
        Ok(())
    }

    pub fn close_executed(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        terminal_evidence: &TerminalEvidenceV1,
    ) -> Result<(), ActionFenceMutationError> {
        self.transition_terminal(
            action_key,
            attempt_identity,
            owner_token_digest,
            terminal_evidence,
            AttemptRecordState::Executed,
        )
    }

    pub fn record_provider_entry_proof(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        proof_digest: String,
    ) -> Result<(), ActionFenceMutationError> {
        if proof_digest.trim().is_empty() || proof_digest.len() > MAX_REF_LEN {
            return Err(ActionFenceMutationError::InvalidTransition);
        }

        let current = self
            .attempts
            .get_mut(attempt_identity.digest())
            .ok_or(ActionFenceMutationError::NotOwner)?;
        if current.state != AttemptRecordState::DispatchPending
            || current.action_key_digest != action_key.digest()
            || current.ownership_token_digest != owner_token_digest
        {
            return Err(ActionFenceMutationError::InvalidTransition);
        }
        if !self.provider_entry_claims.contains_key(attempt_identity.digest()) {
            return Err(ActionFenceMutationError::ProviderEntryClaimMismatch);
        }
        if current.entry_admission_proof_digest.is_some() {
            return Err(ActionFenceMutationError::ProviderEntryClaimed);
        }

        current.entry_admission_proof_digest = Some(proof_digest);
        Ok(())
    }

    pub fn mark_dispatch_pending(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
    ) -> Result<(), ActionFenceMutationError> {
        self.transition_state(
            action_key,
            attempt_identity,
            owner_token_digest,
            AttemptRecordState::DispatchPending,
        )
    }

    pub fn claim_provider_entry(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        claim_token_digest: &str,
    ) -> Result<ProviderEntryClaimV1, ActionFenceMutationError> {
        let current = self
            .attempts
            .get(attempt_identity.digest())
            .ok_or(ActionFenceMutationError::NotOwner)?;
        if current.ownership_token_digest != owner_token_digest {
            return Err(ActionFenceMutationError::OwnershipTokenMismatch);
        }
        if current.action_key_digest != action_key.digest() {
            return Err(ActionFenceMutationError::NotOwner);
        }
        if current.state != AttemptRecordState::DispatchPending {
            return Err(ActionFenceMutationError::InvalidTransition);
        }
        if self.provider_entry_claims.contains_key(attempt_identity.digest()) {
            return Err(ActionFenceMutationError::ProviderEntryClaimed);
        }
        let claim = ProviderEntryClaimV1::new(
            attempt_identity.digest(),
            action_key.digest(),
            owner_token_digest,
            claim_token_digest,
        )
        .map_err(ActionFenceMutationError::StorageFailure)?;
        self.provider_entry_claims
            .insert(attempt_identity.digest().to_owned(), claim.clone());
        Ok(claim)
    }

    pub fn mark_invoked(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
    ) -> Result<(), ActionFenceMutationError> {
        if self.provider_entry_claims.contains_key(attempt_identity.digest()) {
            return Err(ActionFenceMutationError::ProviderEntryClaimMismatch);
        }
        self.transition_state(
            action_key,
            attempt_identity,
            owner_token_digest,
            AttemptRecordState::Invoked,
        )
    }

    pub fn mark_invoked_with_claim(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        claim_token_digest: &str,
    ) -> Result<(), ActionFenceMutationError> {
        let claim = self
            .provider_entry_claims
            .get(attempt_identity.digest())
            .ok_or(ActionFenceMutationError::ProviderEntryClaimMismatch)?;
        if claim.action_key_digest != action_key.digest()
            || claim.owner_token_digest != owner_token_digest
            || claim.claim_token_digest != claim_token_digest
        {
            return Err(ActionFenceMutationError::ProviderEntryClaimMismatch);
        }
        self.transition_state(
            action_key,
            attempt_identity,
            owner_token_digest,
            AttemptRecordState::Invoked,
        )?;
        self.provider_entry_claims.remove(attempt_identity.digest());
        Ok(())
    }

    pub fn record_provider_entry_proof(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        proof_digest: String,
    ) -> Result<(), ActionFenceMutationError> {
        if proof_digest.trim().is_empty() || proof_digest.len() > MAX_REF_LEN {
            return Err(ActionFenceMutationError::InvalidTransition);
        }
        let has_claim = self.provider_entry_claims.contains_key(attempt_identity.digest());
        let current = self
            .attempts
            .get_mut(attempt_identity.digest())
            .ok_or(ActionFenceMutationError::NotOwner)?;
        if current.state != AttemptRecordState::DispatchPending
            || current.action_key_digest != action_key.digest()
            || current.ownership_token_digest != owner_token_digest
        {
            return Err(ActionFenceMutationError::InvalidTransition);
        }
        if !has_claim {
            return Err(ActionFenceMutationError::ProviderEntryClaimMismatch);
        }
        if current.entry_admission_proof_digest.is_some() {
            return Err(ActionFenceMutationError::ProviderEntryProofAlreadyRecorded);
        }
        current.entry_admission_proof_digest = Some(proof_digest);
        Ok(())
    }

    pub fn release_provider_entry_claim_not_entered(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        claim_token_digest: &str,
        marker: String,
    ) -> Result<(), ActionFenceMutationError> {
        if marker.trim().is_empty() || marker.len() > MAX_REF_LEN {
            return Err(ActionFenceMutationError::InvalidTransition);
        }

        let claim = self
            .provider_entry_claims
            .get(attempt_identity.digest())
            .ok_or(ActionFenceMutationError::ProviderEntryClaimMismatch)?;
        if claim.action_key_digest != action_key.digest()
            || claim.owner_token_digest != owner_token_digest
            || claim.claim_token_digest != claim_token_digest
        {
            return Err(ActionFenceMutationError::ProviderEntryClaimMismatch);
        }

        let current = self
            .attempts
            .get_mut(attempt_identity.digest())
            .ok_or(ActionFenceMutationError::NotOwner)?;
        if current.state != AttemptRecordState::DispatchPending
            || current.action_key_digest != action_key.digest()
            || current.ownership_token_digest != owner_token_digest
        {
            return Err(ActionFenceMutationError::InvalidTransition);
        }

        current.state = AttemptRecordState::NotEntered;
        current.not_entered_marker = Some(marker);
        current.entry_admission_proof_digest = None;
        self.provider_entry_claims.remove(attempt_identity.digest());
        self.fences.remove(action_key.digest());
        Ok(())
    }

    pub fn recover_provider_entry_claim(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        claim_token_digest: &str,
    ) -> Result<String, ActionFenceMutationError> {
        let claim = self
            .provider_entry_claims
            .get(attempt_identity.digest())
            .ok_or(ActionFenceMutationError::ProviderEntryClaimMismatch)?;
        if claim.action_key_digest != action_key.digest()
            || claim.owner_token_digest != owner_token_digest
            || claim.claim_token_digest != claim_token_digest
        {
            return Err(ActionFenceMutationError::ProviderEntryClaimMismatch);
        }

        let current = self
            .attempts
            .get_mut(attempt_identity.digest())
            .ok_or(ActionFenceMutationError::NotOwner)?;
        if current.action_key_digest != action_key.digest()
            || current.ownership_token_digest != owner_token_digest
            || current.state != AttemptRecordState::DispatchPending
        {
            return Err(ActionFenceMutationError::InvalidTransition);
        }

        current.state = AttemptRecordState::Indeterminate;
        current.reconciliation_token_digest = Some(reconciliation_token(
            owner_token_digest,
            attempt_identity.digest(),
            action_key.digest(),
        ));
        let token = current
            .reconciliation_token_digest
            .clone()
            .ok_or(ActionFenceMutationError::InvalidTransition)?;
        self.provider_entry_claims.remove(attempt_identity.digest());
        Ok(token)
    }

    pub fn mark_indeterminate(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
    ) -> Result<String, ActionFenceMutationError> {
        if self.provider_entry_claims.contains_key(attempt_identity.digest()) {
            return Err(ActionFenceMutationError::ProviderEntryClaimed);
        }
        self.transition_state(
            action_key,
            attempt_identity,
            owner_token_digest,
            AttemptRecordState::Indeterminate,
        )?;
        self.attempts
            .get(attempt_identity.digest())
            .and_then(|attempt| attempt.reconciliation_token_digest.clone())
            .ok_or(ActionFenceMutationError::InvalidTransition)
    }

    fn transition_state(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        next_state: AttemptRecordState,
    ) -> Result<(), ActionFenceMutationError> {
        let current = self
            .attempts
            .get(attempt_identity.digest())
            .ok_or(ActionFenceMutationError::NotOwner)?;

        if current.ownership_token_digest != owner_token_digest {
            return Err(ActionFenceMutationError::OwnershipTokenMismatch);
        }
        if current.action_key_digest != action_key.digest() {
            return Err(ActionFenceMutationError::NotOwner);
        }

        let current_state = current.state;
        if matches!(
            current_state,
            AttemptRecordState::Executed
                | AttemptRecordState::Failed
                | AttemptRecordState::NotEntered
        ) {
            return Err(ActionFenceMutationError::AlreadyClosed);
        }

        if !current_state.allows_transition_to(next_state) {
            return Err(ActionFenceMutationError::InvalidTransition);
        }

        if matches!(
            next_state,
            AttemptRecordState::DispatchPending
                | AttemptRecordState::Invoked
                | AttemptRecordState::Indeterminate
        ) && (current.provider_reference_seed_digest.is_none()
            || current.provider_reference_descriptor_digest.is_none())
        {
            return Err(ActionFenceMutationError::InvalidTransition);
        }

        let fence = self
            .fences
            .get(action_key.digest())
            .ok_or(ActionFenceMutationError::NotOccupied)?;
        if fence.owner_attempt_identity != attempt_identity.digest() {
            return Err(ActionFenceMutationError::NotOwner);
        }
        if fence.owner_token_digest != owner_token_digest {
            return Err(ActionFenceMutationError::OwnershipTokenMismatch);
        }
        if fence.state.is_closed() {
            return Err(ActionFenceMutationError::AlreadyClosed);
        }

        let attempt = self
            .attempts
            .get_mut(attempt_identity.digest())
            .ok_or(ActionFenceMutationError::NotOwner)?;
        attempt.state = next_state;
        if next_state == AttemptRecordState::Indeterminate {
            attempt.reconciliation_token_digest = Some(reconciliation_token(
                owner_token_digest,
                attempt_identity.digest(),
                action_key.digest(),
            ));
        } else {
            attempt.reconciliation_token_digest = None;
        }

        Ok(())
    }

    fn transition_terminal(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        terminal_evidence: &TerminalEvidenceV1,
        next_state: AttemptRecordState,
    ) -> Result<(), ActionFenceMutationError> {
        let current = self
            .attempts
            .get(attempt_identity.digest())
            .ok_or(ActionFenceMutationError::NotOwner)?;

        if current.state == AttemptRecordState::Indeterminate {
            if current.reconciliation_token_digest.as_deref() != Some(owner_token_digest) {
                return Err(ActionFenceMutationError::OwnershipTokenMismatch);
            }
        } else if current.ownership_token_digest != owner_token_digest {
            return Err(ActionFenceMutationError::OwnershipTokenMismatch);
        }
        if current.action_key_digest != action_key.digest() {
            return Err(ActionFenceMutationError::NotOwner);
        }
        if !matches!(
            current.state,
            AttemptRecordState::Invoked | AttemptRecordState::Indeterminate
        ) {
            return Err(if current.state.is_terminal() {
                ActionFenceMutationError::AlreadyClosed
            } else {
                ActionFenceMutationError::InvalidTransition
            });
        }

        let expected_outcome = match next_state {
            AttemptRecordState::Executed => TerminalOutcomeV1::Executed,
            AttemptRecordState::Failed => TerminalOutcomeV1::Failed,
            _ => return Err(ActionFenceMutationError::InvalidTransition),
        };

        if terminal_evidence.outcome() != expected_outcome
            || terminal_evidence.action_key_digest() != action_key.digest()
            || terminal_evidence.attempt_identity() != attempt_identity.digest()
            || terminal_evidence.operation_id() != current.operation_id
            || terminal_evidence.native_replay_identity() != current.native_replay_identity
            || terminal_evidence.effecting_target_identity() != current.effecting_target_identity
            || terminal_evidence.provider_environment() != current.provider_environment
            || terminal_evidence.provider_audience() != current.provider_audience
            || terminal_evidence.adapter_identity() != current.adapter_identity
        {
            return Err(ActionFenceMutationError::TerminalEvidenceMismatch);
        }

        let fence = self
            .fences
            .get(action_key.digest())
            .ok_or(ActionFenceMutationError::NotOccupied)?;
        if fence.owner_attempt_identity != attempt_identity.digest() {
            return Err(ActionFenceMutationError::NotOwner);
        }
        let expected_fence_token = if current.state == AttemptRecordState::Indeterminate {
            current.ownership_token_digest.as_str()
        } else {
            owner_token_digest
        };
        if fence.owner_token_digest != expected_fence_token {
            return Err(ActionFenceMutationError::OwnershipTokenMismatch);
        }
        if fence.state.is_closed() {
            return Err(ActionFenceMutationError::AlreadyClosed);
        }

        let attempt = self
            .attempts
            .get_mut(attempt_identity.digest())
            .ok_or(ActionFenceMutationError::NotOwner)?;
        attempt.terminal_evidence_digest = Some(terminal_evidence.digest().to_owned());
        attempt.reconciliation_token_digest = None;
        attempt.state = next_state;

        if next_state == AttemptRecordState::Executed {
            let fence = self
                .fences
                .get_mut(action_key.digest())
                .ok_or(ActionFenceMutationError::NotOccupied)?;
            fence.state = ActionFenceState::Closed;
        } else {
            self.fences.remove(action_key.digest());
        }

        Ok(())
    }

    /// Sanity-check all cross-record bindings maintained by the model.
    pub fn validate_invariants(&self) -> Result<(), String> {
        for (attempt_id, record) in &self.attempts {
            record.validate()?;
            if attempt_id != &record.attempt_identity {
                return Err("attempt map key mismatch".into());
            }

            match record.state {
                state if state.occupies_action_fence() => {
                    let fence = self
                        .fences
                        .get(&record.action_key_digest)
                        .ok_or_else(|| "occupied attempt is missing its action fence".to_string())?;
                    if fence.owner_attempt_identity != record.attempt_identity {
                        return Err("action fence owner mismatch".into());
                    }
                    if fence.owner_token_digest != record.ownership_token_digest {
                        return Err("action fence ownership token mismatch".into());
                    }
                    if fence.state != ActionFenceState::Occupied {
                        return Err("non-terminal attempt must have an occupied action fence".into());
                    }
                }
                AttemptRecordState::Executed => {
                    let fence = self
                        .fences
                        .get(&record.action_key_digest)
                        .ok_or_else(|| "executed attempt is missing its closed action fence".to_string())?;
                    if fence.owner_attempt_identity != record.attempt_identity
                        || fence.owner_token_digest != record.ownership_token_digest
                        || fence.state != ActionFenceState::Closed
                    {
                        return Err("executed attempt must own its closed action fence".into());
                    }
                }
                AttemptRecordState::Failed | AttemptRecordState::NotEntered => {
                    if self.fences.contains_key(&record.action_key_digest) {
                        return Err("released terminal attempt still has an action fence".into());
                    }
                }
            }
        }

        for (record) in self.attempts.values() {
            let binding = self
                .replay_bindings
                .get(&record.native_replay_identity)
                .ok_or_else(|| "attempt is missing native replay binding".to_string())?;
            if binding.operation_id != record.operation_id
                || binding.action_key_digest != record.action_key_digest
            {
                return Err("native replay binding does not match attempt record".into());
            }
        }

        for (replay_id, binding) in &self.replay_bindings {
            binding.validate()?;
            if replay_id != &binding.native_replay_identity {
                return Err("native replay binding map key mismatch".into());
            }
            let matching = self.attempts.values().find(|attempt| {
                attempt.native_replay_identity == binding.native_replay_identity
            });
            let attempt = matching
                .ok_or_else(|| "native replay binding references no attempt".to_string())?;
            if attempt.operation_id != binding.operation_id
                || attempt.action_key_digest != binding.action_key_digest
            {
                return Err("native replay binding references mismatched operation/action".into());
            }
        }

        for (attempt_id, claim) in &self.provider_entry_claims {
            claim.validate()?;
            if attempt_id != &claim.attempt_identity {
                return Err("provider entry claim map key mismatch".into());
            }
            let attempt = self
                .attempts
                .get(attempt_id)
                .ok_or_else(|| "provider entry claim references unknown attempt".to_string())?;
            if attempt.state != AttemptRecordState::DispatchPending {
                return Err("provider entry claim requires DISPATCH_PENDING attempt".into());
            }
            if attempt.action_key_digest != claim.action_key_digest
                || attempt.ownership_token_digest != claim.owner_token_digest
            {
                return Err("provider entry claim does not match attempt owner/action".into());
            }
            if let Some(proof_digest) = attempt.entry_admission_proof_digest.as_deref() {
                if proof_digest.trim().is_empty() {
                    return Err("provider entry proof digest is empty".into());
                }
            }
            if attempt.state == AttemptRecordState::DispatchPending
                && attempt.entry_admission_proof_digest.is_some()
                && !self.provider_entry_claims.contains_key(&attempt.attempt_identity)
            {
                return Err(
                    "DISPATCH_PENDING attempt with entry proof is missing its provider entry claim"
                        .into(),
                );
            }
            let fence = self
                .fences
                .get(&claim.action_key_digest)
                .ok_or_else(|| "provider entry claim is missing its action fence".to_string())?;
            if fence.owner_attempt_identity != claim.attempt_identity
                || fence.owner_token_digest != claim.owner_token_digest
                || fence.state != ActionFenceState::Occupied
            {
                return Err("provider entry claim does not match occupied action fence".into());
            }
        }

        for (record) in self.attempts.values() {
            if record.authorization_admission_proof_digest.is_none() {
                return Err("persisted attempt is missing authorization admission proof".into());
            }
        }

        for (action_key, fence) in &self.fences {
            fence.validate()?;
            if action_key != &fence.action_key_digest {
                return Err("fence map key mismatch".into());
            }
            let attempt = self
                .attempts
                .get(&fence.owner_attempt_identity)
                .ok_or_else(|| "fence references unknown attempt".to_string())?;
            if attempt.action_key_digest != *action_key {
                return Err("fence action key does not match attempt record".into());
            }
            if fence.state == ActionFenceState::Occupied
                && !attempt.state.occupies_action_fence()
            {
                return Err("occupied fence references a non-occupying attempt".into());
            }
            if fence.state == ActionFenceState::Closed
                && attempt.state != AttemptRecordState::Executed
            {
                return Err("closed fence must reference an Executed attempt".into());
            }
        }

        Ok(())
    }
}


impl DurableActionFenceStore for AtomicActionFenceModelV1 {
    fn atomically_admit(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        record: AttemptRecordV1,
        authorization_proof: AuthorizationAdmissionProofV1,
    ) -> Result<AtomicAdmissionDecision, String> {
        self.admit(
            action_key,
            attempt_identity,
            record,
            authorization_proof,
        )
    }

    fn atomically_record_provider_entry_proof(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        proof_digest: String,
    ) -> Result<(), ActionFenceMutationError> {
        self.record_provider_entry_proof(
            action_key,
            attempt_identity,
            owner_token_digest,
            proof_digest,
        )
    }

    fn atomically_mark_dispatch_pending(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
    ) -> Result<(), ActionFenceMutationError> {
        self.mark_dispatch_pending(
            action_key,
            attempt_identity,
            owner_token_digest,
        )
    }

    fn atomically_claim_provider_entry(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        claim_token_digest: &str,
    ) -> Result<ProviderEntryClaimV1, ActionFenceMutationError> {
        self.claim_provider_entry(
            action_key,
            attempt_identity,
            owner_token_digest,
            claim_token_digest,
        )
    }

    fn atomically_mark_invoked(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        claim_token_digest: &str,
    ) -> Result<(), ActionFenceMutationError> {
        self.mark_invoked_with_claim(
            action_key,
            attempt_identity,
            owner_token_digest,
            claim_token_digest,
        )
    }

    fn atomically_release_provider_entry_claim_not_entered(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        claim_token_digest: &str,
        marker: String,
    ) -> Result<(), ActionFenceMutationError> {
        self.release_provider_entry_claim_not_entered(
            action_key,
            attempt_identity,
            owner_token_digest,
            claim_token_digest,
            marker,
        )
    }

    fn atomically_recover_provider_entry_claim(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        claim_token_digest: &str,
    ) -> Result<String, ActionFenceMutationError> {
        self.recover_provider_entry_claim(
            action_key,
            attempt_identity,
            owner_token_digest,
            claim_token_digest,
        )
    }

    fn atomically_mark_indeterminate(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
    ) -> Result<String, ActionFenceMutationError> {
        self.mark_indeterminate(
            action_key,
            attempt_identity,
            owner_token_digest,
        )
    }

    fn atomically_release_after_failed(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        terminal_evidence: &TerminalEvidenceV1,
    ) -> Result<(), ActionFenceMutationError> {
        self.release_after_failed(
            action_key,
            attempt_identity,
            owner_token_digest,
            terminal_evidence,
        )
    }

    fn atomically_close_executed(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        terminal_evidence: &TerminalEvidenceV1,
    ) -> Result<(), ActionFenceMutationError> {
        self.close_executed(
            action_key,
            attempt_identity,
            owner_token_digest,
            terminal_evidence,
        )
    }

    fn atomically_release_not_entered(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        marker: String,
    ) -> Result<(), ActionFenceMutationError> {
        self.release_not_entered(action_key, attempt_identity, owner_token_digest, marker)
    }

    fn durably_read_attempt(
        &self,
        attempt_identity: &AttemptIdentityV1,
    ) -> Result<Option<AttemptRecordV1>, String> {
        Ok(self.attempts.get(attempt_identity.digest()).cloned())
    }

    fn durably_read_fence(
        &self,
        action_key: &ActionKeyV1,
    ) -> Result<Option<ActionFenceRecordV1>, String> {
        Ok(self.fences.get(action_key.digest()).cloned())
    }

    fn durably_read_replay_binding(
        &self,
        native_replay_identity: &str,
    ) -> Result<Option<NativeReplayBindingV1>, String> {
        Ok(self.replay_bindings.get(native_replay_identity).cloned())
    }

    fn durably_read_provider_entry_claim(
        &self,
        attempt_identity: &AttemptIdentityV1,
    ) -> Result<Option<ProviderEntryClaimV1>, String> {
        Ok(self.provider_entry_claims.get(attempt_identity.digest()).cloned())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key() -> ActionKeyV1 {
        ActionKeyV1::new("relying-party", "target-1", "action-1").unwrap()
    }

    fn attempt(id: &str) -> AttemptIdentityV1 {
        AttemptIdentityV1::new("payments", "boundary-1", id).unwrap()
    }

    fn model_admit(
        model: &mut AtomicActionFenceModelV1,
        action: &ActionKeyV1,
        owner: &AttemptIdentityV1,
        record: AttemptRecordV1,
    ) -> Result<AtomicAdmissionDecision, String> {
        let proof = AuthorizationAdmissionProofV1::new(
            &record,
            action,
            0,
            u64::MAX,
            "test-authorization-snapshot",
            "test-policy-snapshot",
            "test-status-snapshot",
            "test-admission-verifier-v1",
        )?;
        model.admit(action, owner, record, proof)
    }

    fn terminal_evidence(outcome: TerminalOutcomeV1, attempt_id: &str) -> TerminalEvidenceV1 {
        let action = key();
        let owner = attempt(attempt_id);
        let record = record(
            attempt_id,
            "operation-1",
            AttemptRecordState::Invoked,
        );
        TerminalEvidenceV1::from_attempt(
            &action,
            &record,
            outcome,
            format!("provider-idempotency-{attempt_id}"),
            format!("provider-evidence-{attempt_id}"),
            "qualified-verifier-v1",
        )
        .unwrap()
    }
    fn record(id: &str, operation: &str, state: AttemptRecordState) -> AttemptRecordV1 {
        let key = key();
        AttemptRecordV1::new(
            &attempt(id),
            operation,
            format!("native-replay-{operation}"),
            key.material_action_digest(),
            &key,
            Some("provider-seed-1".into()),
            Some("provider-descriptor-1".into()),
            "stripe-live-account-1",
            "payments-audience-1",
            "payments-adapter-v1",
            format!("owner-token-{id}"),
            state,
        )
        .unwrap()
    }

    #[test]
    fn persisted_provider_idempotency_key_is_versioned_and_fail_closed() {
        let action = key();
        let record = record(
            "attempt-provider-key",
            "operation-provider-key",
            AttemptRecordState::Consumed,
        );

        let derived = derive_provider_idempotency_key(
            &record.native_replay_identity,
            &record.action_key_digest,
            &record.provider_environment,
            &record.provider_audience,
            &record.adapter_identity,
        )
        .unwrap();
        assert_eq!(record.provider_idempotency_key, derived);

        let mut tampered = record.clone();
        tampered.provider_idempotency_key = "constitutional-provider-idempotency-v1:tampered".into();
        assert!(tampered.validate().is_err());
        assert_ne!(
            tampered.record_digest(),
            record.record_digest(),
        );
        assert_eq!(action.digest(), record.action_key_digest);
    }

    #[test]
    fn terminal_evidence_digest_binds_provider_idempotency_key() {
        let action = key();
        let owner = attempt("attempt-idempotency-binding");
        let record = record(
            "attempt-idempotency-binding",
            "operation-idempotency-binding",
            AttemptRecordState::Invoked,
        );

        let first = TerminalEvidenceV1::from_attempt(
            &action,
            &record,
            TerminalOutcomeV1::Executed,
            "provider-idempotency-a",
            "provider-evidence",
            "qualified-verifier-v1",
        )
        .unwrap();
        let second = TerminalEvidenceV1::from_attempt(
            &action,
            &record,
            TerminalOutcomeV1::Executed,
            "provider-idempotency-b",
            "provider-evidence",
            "qualified-verifier-v1",
        )
        .unwrap();

        assert_ne!(first.digest(), second.digest());
        assert_eq!(first.provider_idempotency_key(), "provider-idempotency-a");
        assert_eq!(second.provider_idempotency_key(), "provider-idempotency-b");
        assert_eq!(first.attempt_identity(), owner.digest());
    }

    #[test]
    fn state_transition_contract_is_exhaustive_and_fail_closed() {
        let states = [
            AttemptRecordState::Consumed,
            AttemptRecordState::Reserved,
            AttemptRecordState::DispatchPending,
            AttemptRecordState::Invoked,
            AttemptRecordState::Executed,
            AttemptRecordState::Failed,
            AttemptRecordState::Indeterminate,
            AttemptRecordState::NotEntered,
        ];

        for current in states {
            for next in states {
                let expected = matches!(
                    (current, next),
                    (AttemptRecordState::Consumed, AttemptRecordState::DispatchPending)
                        | (AttemptRecordState::Reserved, AttemptRecordState::DispatchPending)
                        | (AttemptRecordState::DispatchPending, AttemptRecordState::Invoked)
                        | (AttemptRecordState::DispatchPending, AttemptRecordState::Indeterminate)
                        | (AttemptRecordState::Invoked, AttemptRecordState::Indeterminate)
                );
                assert_eq!(
                    current.allows_transition_to(next),
                    expected,
                    "unexpected transition contract: {:?} -> {:?}",
                    current,
                    next
                );
            }
        }

        for state in states {
            assert_eq!(
                state.allows_not_entered_release(),
                matches!(
                    state,
                    AttemptRecordState::Consumed | AttemptRecordState::Reserved
                )
            );
        }
    }

    #[test]
    fn provider_context_is_historical_and_required_before_dispatch() {
        let key = key();
        let owner = attempt("attempt-1");
        let record = AttemptRecordV1::new(
            &owner,
            "operation-1",
            "native-replay-constructor-test",
            key.material_action_digest(),
            &key,
            None,
            None,
            "stripe-live-account-1",
            "payments-audience-1",
            "payments-adapter-v1",
            "owner-token-attempt-1",
            AttemptRecordState::Consumed,
        )
        .unwrap();

        let mut model = AtomicActionFenceModelV1::new();
        model_admit(&mut model, &key, &owner, record).unwrap();

        assert_eq!(
            model.mark_dispatch_pending(&key, &owner, "owner-token-attempt-1"),
            Err(ActionFenceMutationError::InvalidTransition)
        );
        assert_eq!(model.fence(key.digest()).is_some(), true);
    }

    #[test]
    fn record_has_distinct_typed_slots() {
        let a = record("attempt-1", "operation-1", AttemptRecordState::Consumed);
        assert_ne!(a.operation_id, a.attempt_identity);
        assert_ne!(a.native_replay_identity, a.action_key_digest);
        assert_ne!(a.action_digest, a.effecting_target_identity);
        assert!(a.provider_reference_seed_digest.is_some());
        assert!(a.provider_reference_descriptor_digest.is_some());
    }

    #[test]
    fn native_replay_binding_digest_is_deterministic() {
        let a = NativeReplayBindingV1::new(
            "native-replay-1",
            "operation-1",
            key().digest(),
        )
        .unwrap();
        let b = NativeReplayBindingV1::new(
            "native-replay-1",
            "operation-1",
            key().digest(),
        )
        .unwrap();

        assert_eq!(a.digest(), b.digest());
        assert!(a.digest().starts_with(NATIVE_REPLAY_BINDING_PREFIX));
    }

    #[test]
    fn native_replay_identity_has_its_own_collision_namespace() {
        let mut model = AtomicActionFenceModelV1::new();
        let key = key();
        let first_owner = attempt("attempt-1");
        model
            .admit(
                &key,
                &first_owner,
                record("attempt-1", "operation-1", AttemptRecordState::Consumed),
            )
            .unwrap();

        let mut second = record("attempt-2", "operation-2", AttemptRecordState::Consumed);
        second.native_replay_identity = "native-replay-operation-1".into();
        second.action_key_digest = key.digest().into();
        assert!(matches!(
            model_admit(&mut model, &key, &attempt("attempt-2"), second).unwrap(),
            AtomicAdmissionDecision::NativeReplayConflict { .. }
        ));
        assert!(model.attempt("constitutional-attempt-identity-v1:attempt-2").is_none());
    }

    #[test]
    fn same_native_replay_identity_can_span_retries_of_one_operation_and_action() {
        let mut model = AtomicActionFenceModelV1::new();
        let key = key();
        let first_owner = attempt("attempt-1");
        model
            .admit(
                &key,
                &first_owner,
                record("attempt-1", "operation-1", AttemptRecordState::Consumed),
            )
            .unwrap();
        model
            .mark_dispatch_pending(&key, &first_owner, "owner-token-attempt-1")
            .unwrap();
        model
            .mark_invoked(&key, &first_owner, "owner-token-attempt-1")
            .unwrap();
        model
            .release_after_failed(
                &key,
                &first_owner,
                "owner-token-attempt-1",
                &terminal_evidence(TerminalOutcomeV1::Failed, "attempt-1"),
            )
            .unwrap();

        let second_owner = attempt("attempt-2");
        assert_eq!(
            model
                .admit(
                    &key,
                    &second_owner,
                    record("attempt-2", "operation-1", AttemptRecordState::Consumed),
                )
                .unwrap(),
            AtomicAdmissionDecision::Admitted
        );
        assert!(model.validate_invariants().is_ok());
    }

    #[test]
    fn admission_is_same_action_not_operation_id() {
        let mut model = AtomicActionFenceModelV1::new();
        assert_eq!(
            model
                .admit(
                    &key(),
                    &attempt("attempt-1"),
                    record("attempt-1", "operation-1", AttemptRecordState::Consumed),
                )
                .unwrap(),
            AtomicAdmissionDecision::Admitted
        );

        let second = record("attempt-2", "operation-2", AttemptRecordState::Consumed);
        assert_eq!(
            model
                .admit(&key(), &attempt("attempt-2"), second)
                .unwrap(),
            AtomicAdmissionDecision::ActionInFlight
        );
        assert!(model.validate_invariants().is_ok());
    }

    #[test]
    fn typed_roots_must_match_persisted_record() {
        let mut model = AtomicActionFenceModelV1::new();
        let key_a = key();
        let key_b = ActionKeyV1::new("relying-party", "target-1", "action-2").unwrap();
        let owner = attempt("attempt-1");
        let record = record("attempt-1", "operation-1", AttemptRecordState::Consumed);

        assert!(model_admit(&mut model, &key_b, &owner, record.clone()).is_err());
        assert!(model_admit(&mut model, &key_a, &attempt("attempt-2"), record).is_err());
        assert!(model.fence(key_a.digest()).is_none());
        assert!(model.attempt(owner.digest()).is_none());
    }

    #[test]
    fn conflicting_admission_is_atomic_and_leaves_prior_state_unchanged() {
        let mut model = AtomicActionFenceModelV1::new();
        model
            .admit(
                &key(),
                &attempt("attempt-1"),
                record(
                    "attempt-1",
                    "operation-1",
                    AttemptRecordState::Consumed,
                ),
            )
            .unwrap();

        let before_attempt = model
            .attempt("constitutional-attempt-identity-v1:attempt-1")
            .unwrap()
            .record_digest();
        let before_fence = model.fence(key().digest()).unwrap().record_digest();

        assert_eq!(
            model
                .admit(
                    &key(),
                    &attempt("attempt-2"),
                    record(
                        "attempt-2",
                        "operation-2",
                        AttemptRecordState::Consumed,
                    ),
                )
                .unwrap(),
            AtomicAdmissionDecision::ActionInFlight
        );

        assert_eq!(
            model
                .attempt("constitutional-attempt-identity-v1:attempt-1")
                .unwrap()
                .record_digest(),
            before_attempt
        );
        assert_eq!(model.fence(key().digest()).unwrap().record_digest(), before_fence);
        assert!(model.attempt("constitutional-attempt-identity-v1:attempt-2").is_none());
    }

    #[test]
    fn different_action_keys_can_be_admitted_concurrently() {
        let mut model = AtomicActionFenceModelV1::new();
        let first = record("attempt-1", "operation-1", AttemptRecordState::Consumed);
        let mut second = record("attempt-2", "operation-2", AttemptRecordState::Consumed);
        second.action_key_digest =
            ActionKeyV1::new("relying-party", "target-1", "action-2").unwrap().digest().into();
        second.action_digest = "action-2".into();
        second.effecting_target_identity = "target-1".into();
        second.validate().unwrap();

        let key1 = key();
        let key2 = ActionKeyV1::new("relying-party", "target-1", "action-2").unwrap();
        let attempt1 = attempt("attempt-1");
        let attempt2 = attempt("attempt-2");
        assert_eq!(
            model_admit(&mut model, &key1, &attempt1, first).unwrap(),
            AtomicAdmissionDecision::Admitted
        );
        assert_eq!(
            model_admit(&mut model, &key2, &attempt2, second).unwrap(),
            AtomicAdmissionDecision::Admitted
        );
        assert!(model.validate_invariants().is_ok());
    }

    #[test]
    fn fresh_authority_and_new_operation_cannot_bypass_fence() {
        let mut model = AtomicActionFenceModelV1::new();
        model
            .admit(
                &key(),
                &attempt("attempt-1"),
                record("attempt-1", "operation-1", AttemptRecordState::Consumed),
            )
            .unwrap();
        model
            .mark_dispatch_pending(&key(), &attempt("attempt-1"), "owner-token-attempt-1")
            .unwrap();
        model
            .mark_invoked(&key(), &attempt("attempt-1"), "owner-token-attempt-1")
            .unwrap();
        model
            .mark_indeterminate(&key(), &attempt("attempt-1"), "owner-token-attempt-1")
            .unwrap();

        let mut second = record("attempt-2", "operation-2", AttemptRecordState::Consumed);
        second.native_replay_identity = "native-replay-2".into();

        assert_eq!(
            model
                .admit(&key(), &attempt("attempt-2"), second)
                .unwrap(),
            AtomicAdmissionDecision::ActionInFlight
        );
    }

    #[test]
    fn executed_closes_action_for_new_attempts() {
        let mut model = AtomicActionFenceModelV1::new();
        let first_attempt = attempt("attempt-1");
        let first = record("attempt-1", "operation-1", AttemptRecordState::Consumed);
        model
            .admit(&key(), &attempt("attempt-1"), first)
            .unwrap();

        model
            .mark_dispatch_pending(&key(), &first_attempt, "owner-token-attempt-1")
            .unwrap();
        model
            .mark_invoked(&key(), &first_attempt, "owner-token-attempt-1")
            .unwrap();
        model
            .close_executed(
                &key(),
                &first_attempt,
                "owner-token-attempt-1",
                &terminal_evidence(TerminalOutcomeV1::Executed, "attempt-1"),
            )
            .unwrap();

        assert_eq!(
            model
                .admit(
                    &key(),
                    &attempt("attempt-2"),
                    record("attempt-2", "operation-2", AttemptRecordState::Consumed),
                )
                .unwrap(),
            AtomicAdmissionDecision::ActionAlreadyExecuted
        );
    }

    #[test]
    fn failed_releases_only_after_owner_proof() {
        let mut model = AtomicActionFenceModelV1::new();
        let first_attempt = attempt("attempt-1");
        model
            .admit(
                &key(),
                &attempt("attempt-1"),
                record("attempt-1", "operation-1", AttemptRecordState::Consumed),
            )
            .unwrap();

        assert_eq!(
            model.release_after_failed(
            &key(),
            &first_attempt,
            "wrong-owner",
            &terminal_evidence(TerminalOutcomeV1::Failed, "attempt-1"),
        ),
            Err(ActionFenceMutationError::OwnershipTokenMismatch)
        );
        assert!(model.fence(key().digest()).is_some());

        model
            .mark_dispatch_pending(&key(), &first_attempt, "owner-token-attempt-1")
            .unwrap();
        model
            .mark_invoked(&key(), &first_attempt, "owner-token-attempt-1")
            .unwrap();
        model
            .release_after_failed(
                &key(),
                &first_attempt,
                "owner-token-attempt-1",
                &terminal_evidence(TerminalOutcomeV1::Failed, "attempt-1"),
            )
            .unwrap();

        assert!(model.fence(key().digest()).is_none());
        assert_eq!(
            model
                .admit(
                    &key(),
                    &attempt("attempt-2"),
                    record("attempt-2", "operation-2", AttemptRecordState::Consumed),
                )
                .unwrap(),
            AtomicAdmissionDecision::Admitted
        );
    }

    #[test]
    fn not_entered_requires_explicit_marker_and_releases_fence() {
        let mut model = AtomicActionFenceModelV1::new();
        let first_attempt = attempt("attempt-1");
        model
            .admit(
                &key(),
                &first_attempt,
                record("attempt-1", "operation-1", AttemptRecordState::Reserved),
            )
            .unwrap();

        model
            .release_not_entered(
                &key(),
                &first_attempt,
                "owner-token-attempt-1",
                "not-entered-proof-1",
            )
            .unwrap();

        let attempt = model.attempt(first_attempt.digest()).unwrap();
        assert_eq!(attempt.state, AttemptRecordState::NotEntered);
        assert_eq!(
            attempt.not_entered_marker.as_deref(),
            Some("not-entered-proof-1")
        );
        assert!(model.fence(key().digest()).is_none());
        assert!(model.validate_invariants().is_ok());
    }


    #[test]
    fn dispatch_pending_cannot_be_released_as_not_entered() {
        let mut model = AtomicActionFenceModelV1::new();
        let first_attempt = attempt("attempt-1");
        model
            .admit(
                &key(),
                &first_attempt,
                record(
                    "attempt-1",
                    "operation-1",
                    AttemptRecordState::DispatchPending,
                ),
            )
            .unwrap();

        assert_eq!(
            model.release_not_entered(
                &key(),
                &first_attempt,
                "owner-token-attempt-1",
                "not-entered-proof-1",
            ),
            Err(ActionFenceMutationError::NotOccupied)
        );
        assert!(model.fence(key().digest()).is_some());
    }

    #[test]
    fn indeterminate_rotates_terminal_resolution_token() {
        let mut model = AtomicActionFenceModelV1::new();
        let action = key();
        let owner = attempt("attempt-resolution");
        model
            .admit(
                &action,
                &owner,
                record(
                    "attempt-resolution",
                    "operation-resolution",
                    AttemptRecordState::Consumed,
                ),
            )
            .unwrap();
        model
            .mark_dispatch_pending(&action, &owner, "owner-token-attempt-resolution")
            .unwrap();
        model
            .mark_invoked(&action, &owner, "owner-token-attempt-resolution")
            .unwrap();

        let resolution_token = model
            .mark_indeterminate(&action, &owner, "owner-token-attempt-resolution")
            .unwrap();

        assert_ne!(resolution_token, "owner-token-attempt-resolution");
        assert_eq!(
            model.close_executed(
                &action,
                &owner,
                "owner-token-attempt-resolution",
                &terminal_evidence(TerminalOutcomeV1::Executed, "attempt-resolution"),
            ),
            Err(ActionFenceMutationError::OwnershipTokenMismatch)
        );

        model
            .close_executed(
                &action,
                &owner,
                &resolution_token,
                &terminal_evidence(TerminalOutcomeV1::Executed, "attempt-resolution"),
            )
            .unwrap();
    }

    #[test]
    fn wrong_attempt_cannot_close_or_release_same_action() {
        let mut model = AtomicActionFenceModelV1::new();
        let first_attempt = attempt("attempt-1");
        let second_attempt = attempt("attempt-2");
        model
            .admit(
                &key(),
                &first_attempt,
                record("attempt-1", "operation-1", AttemptRecordState::Consumed),
            )
            .unwrap();

        assert_eq!(
            model.close_executed(
                &key(),
                &second_attempt,
                "owner-token-attempt-2",
                &terminal_evidence(TerminalOutcomeV1::Executed, "attempt-2"),
            ),
            Err(ActionFenceMutationError::NotOwner)
        );
        assert_eq!(
            model.release_after_failed(
                &key(),
                &second_attempt,
                "owner-token-attempt-2",
                &terminal_evidence(TerminalOutcomeV1::Failed, "attempt-2"),
            ),
            Err(ActionFenceMutationError::NotOwner)
        );
        assert!(model.fence(key().digest()).is_some());
    }

    #[test]
    fn duplicate_same_attempt_is_not_a_new_fence_owner() {
        let mut model = AtomicActionFenceModelV1::new();
        let first = record("attempt-1", "operation-1", AttemptRecordState::Consumed);
        assert_eq!(
            model
                .admit(&key(), &attempt("attempt-1"), first.clone())
                .unwrap(),
            AtomicAdmissionDecision::Admitted
        );
        assert_eq!(
            model
                .admit(&key(), &attempt("attempt-1"), first)
                .unwrap(),
            AtomicAdmissionDecision::DuplicateAttempt
        );
        assert!(model.validate_invariants().is_ok());
    }

    #[test]
    fn terminal_attempt_records_require_transition_proof() {
        let key = key();
        assert!(AttemptRecordV1::new(
            &attempt("attempt-1"),
            "operation-1",
            "native-replay-1",
            key.material_action_digest(),
            &key,
            Some("provider-seed-1".into()),
            Some("provider-descriptor-1".into()),
            "stripe-live-account-1",
            "payments-audience-1",
            "payments-adapter-v1",
            "owner-token-attempt-1",
            AttemptRecordState::Executed,
        )
        .is_err());

        assert!(AttemptRecordV1::new(
            &attempt("attempt-2"),
            "operation-2",
            "native-replay-1",
            key.material_action_digest(),
            &key,
            Some("provider-seed-1".into()),
            Some("provider-descriptor-1".into()),
            "stripe-live-account-1",
            "payments-audience-1",
            "payments-adapter-v1",
            "owner-token-attempt-2",
            AttemptRecordState::Failed,
        )
        .is_err());

        assert!(AttemptRecordV1::new(
            &attempt("attempt-3"),
            "operation-3",
            "native-replay-1",
            key.material_action_digest(),
            &key,
            Some("provider-seed-1".into()),
            Some("provider-descriptor-1".into()),
            "stripe-live-account-1",
            "payments-audience-1",
            "payments-adapter-v1",
            "owner-token-attempt-3",
            AttemptRecordState::NotEntered,
        )
        .is_err());
    }


    #[test]
    fn terminal_state_cannot_be_reached_before_invocation() {
        let mut model = AtomicActionFenceModelV1::new();
        let owner = attempt("attempt-1");
        model
            .admit(
                &key(),
                &owner,
                record("attempt-1", "operation-1", AttemptRecordState::Consumed),
            )
            .unwrap();

        assert_eq!(
            model.close_executed(
            &key(),
            &owner,
            "owner-token-attempt-1",
            &terminal_evidence(TerminalOutcomeV1::Executed, "attempt-1"),
        ),
            Err(ActionFenceMutationError::InvalidTransition)
        );
        assert_eq!(
            model.release_after_failed(
            &key(),
            &owner,
            "owner-token-attempt-1",
            &terminal_evidence(TerminalOutcomeV1::Failed, "attempt-1"),
        ),
            Err(ActionFenceMutationError::InvalidTransition)
        );
        assert!(model.fence(key().digest()).is_some());
    }

    #[test]
    fn indeterminate_keeps_fence_until_authoritative_resolution() {
        let mut model = AtomicActionFenceModelV1::new();
        let owner = attempt("attempt-1");
        model
            .admit(
                &key(),
                &owner,
                record("attempt-1", "operation-1", AttemptRecordState::Consumed),
            )
            .unwrap();

        model
            .mark_dispatch_pending(&key(), &owner, "owner-token-attempt-1")
            .unwrap();
        model
            .mark_invoked(&key(), &owner, "owner-token-attempt-1")
            .unwrap();
        model
            .mark_indeterminate(&key(), &owner, "owner-token-attempt-1")
            .unwrap();

        assert_eq!(
            model_admit(&mut model, 
                &key(),
                &attempt("attempt-2"),
                record("attempt-2", "operation-2", AttemptRecordState::Consumed),
            ).unwrap(),
            AtomicAdmissionDecision::ActionInFlight
        );
    }

    #[test]
    fn wrong_terminal_outcome_cannot_close_or_release() {
        let mut model = AtomicActionFenceModelV1::new();
        let owner = attempt("attempt-1");
        model
            .admit(
                &key(),
                &owner,
                record("attempt-1", "operation-1", AttemptRecordState::Consumed),
            )
            .unwrap();
        model
            .mark_dispatch_pending(&key(), &owner, "owner-token-attempt-1")
            .unwrap();
        model
            .mark_invoked(&key(), &owner, "owner-token-attempt-1")
            .unwrap();

        let failed_proof = terminal_evidence(TerminalOutcomeV1::Failed, "attempt-1");
        assert_eq!(
            model.close_executed(
                &key(),
                &owner,
                "owner-token-attempt-1",
                &failed_proof,
            ),
            Err(ActionFenceMutationError::TerminalEvidenceMismatch)
        );
        assert!(model.fence(key().digest()).is_some());
    }

    #[test]
    fn terminal_close_is_replay_stable_but_does_not_reopen() {
        let mut model = AtomicActionFenceModelV1::new();
        let owner = attempt("attempt-1");
        model
            .admit(
                &key(),
                &owner,
                record("attempt-1", "operation-1", AttemptRecordState::Consumed),
            )
            .unwrap();

        model
            .mark_dispatch_pending(&key(), &owner, "owner-token-attempt-1")
            .unwrap();
        model
            .mark_invoked(&key(), &owner, "owner-token-attempt-1")
            .unwrap();
        model
            .close_executed(
                &key(),
                &owner,
                "owner-token-attempt-1",
                &terminal_evidence(TerminalOutcomeV1::Executed, "attempt-1"),
            )
            .unwrap();

        assert_eq!(
            model_admit(&mut model, 
                &key(),
                &attempt("attempt-2"),
                record("attempt-2", "operation-2", AttemptRecordState::Consumed),
            ).unwrap(),
            AtomicAdmissionDecision::ActionAlreadyExecuted
        );
        assert_eq!(
            model.close_executed(
            &key(),
            &owner,
            "owner-token-attempt-1",
            &terminal_evidence(TerminalOutcomeV1::Executed, "attempt-1"),
        ),
            Err(ActionFenceMutationError::AlreadyClosed)
        );
    }

    #[test]
    fn attempt_record_digest_is_deterministic() {
        let a = record("attempt-1", "operation-1", AttemptRecordState::Consumed);
        let b = record("attempt-1", "operation-1", AttemptRecordState::Consumed);
        assert_eq!(a.record_digest(), b.record_digest());
    }
}

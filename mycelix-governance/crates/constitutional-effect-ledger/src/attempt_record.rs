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

pub const ATTEMPT_RECORD_SCHEMA_VERSION: u16 = 1;
pub const ACTION_FENCE_SCHEMA_VERSION: u16 = 1;
pub const ATTEMPT_RECORD_PREFIX: &str = "constitutional-attempt-record-v1:";
pub const ACTION_FENCE_RECORD_PREFIX: &str = "constitutional-action-fence-v1:";
const MAX_ID_LEN: usize = 256;
const MAX_REF_LEN: usize = 512;
const ATTEMPT_RECORD_DOMAIN: &[u8] =
    b"MYCELIX-CONSTITUTIONAL-ATTEMPT-RECORD\0V1\0";
const ACTION_FENCE_RECORD_DOMAIN: &[u8] =
    b"MYCELIX-CONSTITUTIONAL-ACTION-FENCE-RECORD\0V1\0";

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
}

/// Durable attempt record projection.
///
/// Identity-bearing roots are stored as their explicit digests/identifiers rather
/// than as serialized ActionKeyV1 / AttemptIdentityV1 values. The identity types
/// themselves remain non-serializable and private-fielded.
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
    pub ownership_token_digest: String,
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
            ownership_token_digest,
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
        push_str(&mut hasher, &self.ownership_token_digest);
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

    fn validate(&self) -> Result<(), String> {
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
    AttemptOwnershipConflict,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ActionFenceMutationError {
    NotOwner,
    OwnershipTokenMismatch,
    NotOccupied,
    AlreadyClosed,
    InvalidTransition,
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
    ) -> Result<AtomicAdmissionDecision, String>;

    fn atomically_release_after_failed(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
    ) -> Result<(), ActionFenceMutationError>;

    fn atomically_close_executed(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
    ) -> Result<(), ActionFenceMutationError>;

    fn atomically_release_not_entered(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        marker: String,
    ) -> Result<(), ActionFenceMutationError>;
}

/// Reference model for the required atomic admission transition.
///
/// The method admit is deliberately one mutation over the combined durable
/// state model: either the attempt record and action fence are both installed,
/// or neither changes. This demonstrates the conflict theorem without claiming
/// that the in-memory map itself provides crash durability.
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct AtomicActionFenceModelV1 {
    attempts: BTreeMap<String, AttemptRecordV1>,
    fences: BTreeMap<String, ActionFenceRecordV1>,
}

impl AtomicActionFenceModelV1 {
    pub fn new() -> Self {
        Self::default()
    }

    pub fn admit(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        record: AttemptRecordV1,
    ) -> Result<AtomicAdmissionDecision, String> {
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

        if !record.state.occupies_action_fence() {
            return Err("admitted attempt must occupy the action fence".into());
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

    pub fn release_after_failed(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
    ) -> Result<(), ActionFenceMutationError> {
        self.transition_state(
            action_key,
            attempt_identity,
            owner_token_digest,
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
        if !matches!(
            current.state,
            AttemptRecordState::Consumed | AttemptRecordState::Reserved
        ) {
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
    ) -> Result<(), ActionFenceMutationError> {
        self.transition_state(
            action_key,
            attempt_identity,
            owner_token_digest,
            AttemptRecordState::Executed,
        )
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

    pub fn mark_invoked(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
    ) -> Result<(), ActionFenceMutationError> {
        self.transition_state(
            action_key,
            attempt_identity,
            owner_token_digest,
            AttemptRecordState::Invoked,
        )
    }

    pub fn mark_indeterminate(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
    ) -> Result<(), ActionFenceMutationError> {
        self.transition_state(
            action_key,
            attempt_identity,
            owner_token_digest,
            AttemptRecordState::Indeterminate,
        )
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

        let allowed = matches!(
            (current_state, next_state),
            (AttemptRecordState::Consumed, AttemptRecordState::DispatchPending)
                | (AttemptRecordState::Reserved, AttemptRecordState::DispatchPending)
                | (AttemptRecordState::DispatchPending, AttemptRecordState::Invoked)
                | (AttemptRecordState::DispatchPending, AttemptRecordState::Indeterminate)
                | (AttemptRecordState::Invoked, AttemptRecordState::Executed)
                | (AttemptRecordState::Invoked, AttemptRecordState::Failed)
                | (AttemptRecordState::Invoked, AttemptRecordState::Indeterminate)
                | (AttemptRecordState::Indeterminate, AttemptRecordState::Executed)
                | (AttemptRecordState::Indeterminate, AttemptRecordState::Failed)
        );
        if !allowed {
            return Err(ActionFenceMutationError::InvalidTransition);
        }

        if matches!(
            next_state,
            AttemptRecordState::DispatchPending
                | AttemptRecordState::Invoked
                | AttemptRecordState::Executed
                | AttemptRecordState::Failed
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

        match next_state {
            AttemptRecordState::Executed => {
                let fence = self
                    .fences
                    .get_mut(action_key.digest())
                    .ok_or(ActionFenceMutationError::NotOccupied)?;
                fence.state = ActionFenceState::Closed;
            }
            AttemptRecordState::Failed => {
                self.fences.remove(action_key.digest());
            }
            _ => {}
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
    ) -> Result<AtomicAdmissionDecision, String> {
        self.admit(action_key, attempt_identity, record)
    }

    fn atomically_release_after_failed(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
    ) -> Result<(), ActionFenceMutationError> {
        self.release_after_failed(action_key, attempt_identity, owner_token_digest)
    }

    fn atomically_close_executed(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
    ) -> Result<(), ActionFenceMutationError> {
        self.close_executed(action_key, attempt_identity, owner_token_digest)
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

    fn record(id: &str, operation: &str, state: AttemptRecordState) -> AttemptRecordV1 {
        let key = key();
        AttemptRecordV1::new(
            &attempt(id),
            operation,
            "native-replay-1",
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
    fn provider_context_is_historical_and_required_before_dispatch() {
        let key = key();
        let owner = attempt("attempt-1");
        let record = AttemptRecordV1::new(
            &owner,
            "operation-1",
            "native-replay-1",
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
        model.admit(&key, &owner, record).unwrap();

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

        assert!(model.admit(&key_b, &owner, record.clone()).is_err());
        assert!(model.admit(&key_a, &attempt("attempt-2"), record).is_err());
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
            model.admit(&key1, &attempt1, first).unwrap(),
            AtomicAdmissionDecision::Admitted
        );
        assert_eq!(
            model.admit(&key2, &attempt2, second).unwrap(),
            AtomicAdmissionDecision::Admitted
        );
        assert!(model.validate_invariants().is_ok());
    }

    #[test]
    fn fresh_authority_and_new_operation_cannot_bypass_fence() {
        let mut model = AtomicActionFenceModelV1::new();
        let first = record("attempt-1", "operation-1", AttemptRecordState::Indeterminate);
        model.admit(&key(), &attempt("attempt-1"), first).unwrap();

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
            .close_executed(&key(), &first_attempt, "owner-token-attempt-1")
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
            model.release_after_failed(&key(), &first_attempt, "wrong-owner"),
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
            .release_after_failed(&key(), &first_attempt, "owner-token-attempt-1")
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
            model.close_executed(&key(), &second_attempt, "owner-token-attempt-2"),
            Err(ActionFenceMutationError::NotOwner)
        );
        assert_eq!(
            model.release_after_failed(&key(), &second_attempt, "owner-token-attempt-2"),
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
    fn terminal_attempts_cannot_be_admitted_as_occupiers() {
        let mut model = AtomicActionFenceModelV1::new();
        assert!(model
            .admit(
                &key(),
                &attempt("attempt-1"),
                record(
                    "attempt-1",
                    "operation-1",
                    AttemptRecordState::Executed
                ),
            )
            .is_err());
        assert!(model
            .admit(
                &key(),
                &attempt("attempt-2"),
                record(
                    "attempt-2",
                    "operation-2",
                    AttemptRecordState::Failed
                ),
            )
            .is_err());
        assert!(model
            .admit(
                &key(),
                &attempt("attempt-3"),
                record(
                    "attempt-3",
                    "operation-3",
                    AttemptRecordState::NotEntered
                ),
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
            model.close_executed(&key(), &owner, "owner-token-attempt-1"),
            Err(ActionFenceMutationError::InvalidTransition)
        );
        assert_eq!(
            model.release_after_failed(&key(), &owner, "owner-token-attempt-1"),
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
            model.admit(
                &key(),
                &attempt("attempt-2"),
                record("attempt-2", "operation-2", AttemptRecordState::Consumed),
            ).unwrap(),
            AtomicAdmissionDecision::ActionInFlight
        );
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
            .close_executed(&key(), &owner, "owner-token-attempt-1")
            .unwrap();

        assert_eq!(
            model.admit(
                &key(),
                &attempt("attempt-2"),
                record("attempt-2", "operation-2", AttemptRecordState::Consumed),
            ).unwrap(),
            AtomicAdmissionDecision::ActionAlreadyExecuted
        );
        assert_eq!(
            model.close_executed(&key(), &owner, "owner-token-attempt-1"),
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

#![deny(unsafe_code)]

//! Host-side orchestration for the constitutional effect boundary.
//!
//! Holochain supplies provenance, authority, and application APIs. This module
//! supplies the durable pre-provider-entry ordering and must be the only host
//! path that invokes a protected provider.

use crate::SqliteActionFenceStore;
use constitutional_effect_ledger::{
    ActionFenceMutationError, ActionFenceState, ActionKeyV1, AtomicAdmissionDecision,
    AttemptIdentityV1, AttemptRecordState, AttemptRecordV1, DurableActionFenceStore,
    ProviderEntryClaimV1, TerminalEvidenceV1, TerminalOutcomeV1,
};

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProviderObservation {
    Executed { evidence_commitment: String },
    Failed { evidence_commitment: String },
    Indeterminate { evidence_commitment: Option<String> },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VerificationPurpose {
    InitialInvocation,
    Reconciliation,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedTerminalOutcomeV1 {
    outcome: TerminalOutcomeV1,
    evidence_commitment: String,
    verifier_identity: String,
    attempt_identity: String,
    operation_id: String,
    native_replay_identity: String,
    action_key_digest: String,
    provider_idempotency_key: String,
    purpose: VerificationPurpose,
}

impl VerifiedTerminalOutcomeV1 {
    /// Construct a terminal verification proof that is mechanically bound to
    /// the exact attempt and verification purpose supplied to the verifier.
    ///
    /// The boundary still re-checks these bindings before applying the proof.
    pub fn new(
        attempt: &AttemptRecordV1,
        purpose: VerificationPurpose,
        provider_idempotency_key: impl Into<String>,
        outcome: TerminalOutcomeV1,
        evidence_commitment: impl Into<String>,
        verifier_identity: impl Into<String>,
    ) -> Result<Self, String> {
        let provider_idempotency_key = provider_idempotency_key.into();
        let evidence_commitment = evidence_commitment.into();
        let verifier_identity = verifier_identity.into();
        if provider_idempotency_key.trim().is_empty()
            || evidence_commitment.trim().is_empty()
            || verifier_identity.trim().is_empty()
        {
            return Err("verification proof commitments must be non-empty".into());
        }
        Ok(Self {
            outcome,
            evidence_commitment,
            verifier_identity,
            attempt_identity: attempt.attempt_identity.clone(),
            operation_id: attempt.operation_id.clone(),
            native_replay_identity: attempt.native_replay_identity.clone(),
            action_key_digest: attempt.action_key_digest.clone(),
            provider_idempotency_key,
            purpose,
        })
    }

    fn matches(
        &self,
        attempt: &AttemptRecordV1,
        action_key: &ActionKeyV1,
        purpose: VerificationPurpose,
        provider_idempotency_key: &str,
    ) -> bool {
        self.attempt_identity == attempt.attempt_identity
            && self.operation_id == attempt.operation_id
            && self.native_replay_identity == attempt.native_replay_identity
            && self.action_key_digest == action_key.digest()
            && self.provider_idempotency_key == provider_idempotency_key
            && self.purpose == purpose
    }
}

/// The provider receives only the frozen action context required by its
/// downstream interface. Boundary custody, owner tokens, reconciliation tokens,
/// and lifecycle state are intentionally not exposed to the adapter.
pub trait ProviderAdapter {
    /// Provider entry is only exposed through a durable, single-winner permit.
    fn invoke(&mut self, permit: &ProviderEntryPermitV1) -> Result<ProviderObservation, String>;
    fn reconcile(&mut self, context: &ProviderActionContextV1) -> Result<ProviderObservation, String>;
}

/// Frozen action context exposed to the provider adapter.
///
/// This is deliberately smaller than AttemptRecordV1. It contains the material
/// action/provider fields needed for invocation or reconciliation while excluding
/// boundary-only custody state such as owner tokens, durable lifecycle state,
/// reconciliation credentials, and terminal evidence state.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProviderActionContextV1 {
    action_digest: String,
    action_key_digest: String,
    operation_id: String,
    native_replay_identity: String,
    effecting_target_identity: String,
    provider_reference_seed_digest: Option<String>,
    provider_reference_descriptor_digest: Option<String>,
    provider_environment: String,
    provider_audience: String,
    adapter_identity: String,
    provider_idempotency_key: String,
}

impl ProviderActionContextV1 {
    fn from_attempt(
        attempt: &AttemptRecordV1,
        action_key: &ActionKeyV1,
    ) -> Result<Self, BoundaryError> {
        attempt
            .validate()
            .map_err(BoundaryError::Semantic)?;
        if attempt.action_key_digest != action_key.digest()
            || attempt.action_digest != action_key.material_action_digest()
            || attempt.effecting_target_identity != action_key.effecting_target_identity()
        {
            return Err(BoundaryError::Semantic(
                "provider action context does not exactly bind the attempt to the action".into(),
            ));
        }

        let provider_idempotency_key = attempt.provider_idempotency_key().to_owned();
        Ok(Self {
            action_digest: attempt.action_digest.clone(),
            action_key_digest: attempt.action_key_digest.clone(),
            operation_id: attempt.operation_id.clone(),
            native_replay_identity: attempt.native_replay_identity.clone(),
            effecting_target_identity: attempt.effecting_target_identity.clone(),
            provider_reference_seed_digest: attempt.provider_reference_seed_digest.clone(),
            provider_reference_descriptor_digest: attempt.provider_reference_descriptor_digest.clone(),
            provider_environment: attempt.provider_environment.clone(),
            provider_audience: attempt.provider_audience.clone(),
            adapter_identity: attempt.adapter_identity.clone(),
            provider_idempotency_key,
        })
    }

    pub fn action_digest(&self) -> &str {
        &self.action_digest
    }

    pub fn action_key_digest(&self) -> &str {
        &self.action_key_digest
    }

    pub fn operation_id(&self) -> &str {
        &self.operation_id
    }

    pub fn native_replay_identity(&self) -> &str {
        &self.native_replay_identity
    }

    pub fn effecting_target_identity(&self) -> &str {
        &self.effecting_target_identity
    }

    pub fn provider_reference_seed_digest(&self) -> Option<&str> {
        self.provider_reference_seed_digest.as_deref()
    }

    pub fn provider_reference_descriptor_digest(&self) -> Option<&str> {
        self.provider_reference_descriptor_digest.as_deref()
    }

    pub fn provider_environment(&self) -> &str {
        &self.provider_environment
    }

    pub fn provider_audience(&self) -> &str {
        &self.provider_audience
    }

    pub fn adapter_identity(&self) -> &str {
        &self.adapter_identity
    }

    pub fn provider_idempotency_key(&self) -> &str {
        &self.provider_idempotency_key
    }
}

/// Final pre-entry authorization/freshness proof.
///
/// The verifier is responsible for checking the configured authority/policy
/// snapshot and current provider/status state. The boundary binds the returned
/// proof to the exact durable attempt and persisted provider idempotency key,
/// and refuses an expired or otherwise mismatched proof.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FinalProviderEntryProofV1 {
    attempt_identity: String,
    action_key_digest: String,
    operation_id: String,
    native_replay_identity: String,
    provider_idempotency_key: String,
    authorization_snapshot_digest: String,
    status_snapshot_digest: String,
    checked_at_unix_ms: u64,
    valid_until_unix_ms: u64,
    verifier_identity: String,
    digest: String,
}

impl FinalProviderEntryProofV1 {
    pub fn new(
        attempt: &AttemptRecordV1,
        context: &ProviderActionContextV1,
        checked_at_unix_ms: u64,
        valid_until_unix_ms: u64,
        authorization_snapshot_digest: impl Into<String>,
        status_snapshot_digest: impl Into<String>,
        verifier_identity: impl Into<String>,
    ) -> Result<Self, String> {
        let authorization_snapshot_digest = authorization_snapshot_digest.into();
        let status_snapshot_digest = status_snapshot_digest.into();
        let verifier_identity = verifier_identity.into();

        if valid_until_unix_ms <= checked_at_unix_ms {
            return Err("final provider-entry proof validity window is empty".into());
        }
        for (label, value) in [
            ("authorization_snapshot_digest", authorization_snapshot_digest.as_str()),
            ("status_snapshot_digest", status_snapshot_digest.as_str()),
            ("verifier_identity", verifier_identity.as_str()),
        ] {
            if value.trim().is_empty() || value.len() > 512 {
                return Err(format!("{label} must be non-empty and <= 512 bytes"));
            }
        }

        let mut hasher = blake3::Hasher::new();
        hasher.update(b"MYCELIX-CONSTITUTIONAL-FINAL-PROVIDER-ENTRY-PROOF\0V1\0");
        push_digest_string(&mut hasher, &attempt.attempt_identity);
        push_digest_string(&mut hasher, &context.action_key_digest);
        push_digest_string(&mut hasher, &attempt.operation_id);
        push_digest_string(&mut hasher, &attempt.native_replay_identity);
        push_digest_string(&mut hasher, &context.provider_idempotency_key);
        push_digest_string(&mut hasher, &authorization_snapshot_digest);
        push_digest_string(&mut hasher, &status_snapshot_digest);
        hasher.update(&checked_at_unix_ms.to_be_bytes());
        hasher.update(&valid_until_unix_ms.to_be_bytes());
        push_digest_string(&mut hasher, &verifier_identity);

        Ok(Self {
            attempt_identity: attempt.attempt_identity.clone(),
            action_key_digest: context.action_key_digest.clone(),
            operation_id: attempt.operation_id.clone(),
            native_replay_identity: attempt.native_replay_identity.clone(),
            provider_idempotency_key: context.provider_idempotency_key.clone(),
            authorization_snapshot_digest,
            status_snapshot_digest,
            checked_at_unix_ms,
            valid_until_unix_ms,
            verifier_identity,
            digest: format!(
                "constitutional-final-provider-entry-proof-v1:{}",
                hasher.finalize().to_hex()
            ),
        })
    }

    fn matches(
        &self,
        attempt: &AttemptRecordV1,
        context: &ProviderActionContextV1,
    ) -> bool {
        self.attempt_identity == attempt.attempt_identity
            && self.action_key_digest == context.action_key_digest
            && self.operation_id == attempt.operation_id
            && self.native_replay_identity == attempt.native_replay_identity
            && self.provider_idempotency_key == context.provider_idempotency_key
            && self.valid_until_unix_ms > self.checked_at_unix_ms
    }

    fn is_fresh(&self, now_unix_ms: u64) -> bool {
        self.checked_at_unix_ms <= now_unix_ms && now_unix_ms < self.valid_until_unix_ms
    }

    pub fn digest(&self) -> &str {
        &self.digest
    }

    pub fn authorization_snapshot_digest(&self) -> &str {
        &self.authorization_snapshot_digest
    }

    pub fn status_snapshot_digest(&self) -> &str {
        &self.status_snapshot_digest
    }

    pub fn verifier_identity(&self) -> &str {
        &self.verifier_identity
    }
}

pub trait FinalProviderEntryVerifier {
    fn verify(
        &self,
        attempt: &AttemptRecordV1,
        context: &ProviderActionContextV1,
        now_unix_ms: u64,
    ) -> Result<FinalProviderEntryProofV1, String>;
}

/// Final provider-entry capability.
///
/// This object can only be constructed from a durably confirmed
/// DISPATCH_PENDING attempt plus its single-winner provider-entry claim.
/// Boundary-only custody state remains private to this type.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProviderEntryPermitV1 {
    context: ProviderActionContextV1,
    claim: ProviderEntryClaimV1,
    final_entry_proof: FinalProviderEntryProofV1,
}

impl ProviderEntryPermitV1 {
    fn new(
        attempt: AttemptRecordV1,
        action_key: &ActionKeyV1,
        claim: ProviderEntryClaimV1,
        final_entry_proof: FinalProviderEntryProofV1,
        now_unix_ms: u64,
    ) -> Result<Self, BoundaryError> {
        if attempt.state != AttemptRecordState::DispatchPending {
            return Err(BoundaryError::Semantic(
                "provider entry permit requires durable DISPATCH_PENDING".into(),
            ));
        }
        if attempt.attempt_identity != claim.attempt_identity
            || attempt.action_key_digest != claim.action_key_digest
            || attempt.ownership_token_digest != claim.owner_token_digest
            || action_key.digest() != claim.action_key_digest
        {
            return Err(BoundaryError::Semantic(
                "provider entry claim does not exactly bind the attempt and action".into(),
            ));
        }

        let context = ProviderActionContextV1::from_attempt(&attempt, action_key)?;
        if !final_entry_proof.matches(&attempt, &context)
            || !final_entry_proof.is_fresh(now_unix_ms)
        {
            return Err(BoundaryError::Semantic(
                "final provider-entry proof is stale or not bound to the exact attempt"
                    .into(),
            ));
        }
        Ok(Self {
            context,
            claim,
            final_entry_proof,
        })
    }

    pub fn context(&self) -> &ProviderActionContextV1 {
        &self.context
    }

    pub fn provider_idempotency_key(&self) -> &str {
        self.context.provider_idempotency_key()
    }

    pub fn final_entry_proof_digest(&self) -> &str {
        self.final_entry_proof.digest()
    }

    fn claim_token_digest(&self) -> &str {
        &self.claim.claim_token_digest
    }
}

fn push_digest_string(hasher: &mut blake3::Hasher, value: &str) {
    hasher.update(&(value.len() as u64).to_be_bytes());
    hasher.update(value.as_bytes());
}

fn current_unix_ms() -> Result<u64, String> {
    use std::time::{SystemTime, UNIX_EPOCH};
    SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .map(|duration| duration.as_millis() as u64)
        .map_err(|error| format!("trusted clock unavailable: {error}"))
}

fn derive_entry_claim_token(
    attempt_identity: &AttemptIdentityV1,
    owner_token_digest: &str,
    nonce: u64,
) -> String {
    let mut hasher = blake3::Hasher::new();
    hasher.update(b"MYCELIX-CONSTITUTIONAL-PROVIDER-ENTRY-CLAIM-TOKEN\0V1\0");
    push_digest_string(&mut hasher, attempt_identity.digest());
    push_digest_string(&mut hasher, owner_token_digest);
    push_digest_string(&mut hasher, &std::process::id().to_string());
    push_digest_string(&mut hasher, &nonce.to_string());
    format!("constitutional-provider-entry-claim-token-v1:{}", hasher.finalize().to_hex())
}

/// Authentication/authorization for terminal provider evidence remains outside
/// the durable store.
pub trait OutcomeVerifier {
    fn verify(
        &self,
        attempt: &AttemptRecordV1,
        provider_idempotency_key: &str,
        observation: &ProviderObservation,
        purpose: VerificationPurpose,
    ) -> Result<VerifiedTerminalOutcomeV1, String>;
}

/// Exact attempt-scoped pre-entry recovery authorization.
///
/// The authorization must already be authenticated by the configured
/// RecoveryAuthorizer. The boundary additionally checks that every scope field
/// matches the durable attempt before releasing anything.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PreEntryRecoveryAuthorizationV1 {
    pub attempt_identity: String,
    pub boundary_kind: String,
    pub boundary_instance_id: String,
    pub attempt_id: String,
    pub operation_id: String,
    pub native_replay_identity: String,
    pub action_key_digest: String,
    pub authorization_commitment: String,
    pub authorizer_identity: String,
}

impl PreEntryRecoveryAuthorizationV1 {
    pub fn new(
        attempt: &AttemptIdentityV1,
        operation_id: impl Into<String>,
        native_replay_identity: impl Into<String>,
        action_key: &ActionKeyV1,
        authorization_commitment: impl Into<String>,
        authorizer_identity: impl Into<String>,
    ) -> Result<Self, String> {
        let out = Self {
            attempt_identity: attempt.digest().to_owned(),
            boundary_kind: attempt.boundary_kind().to_owned(),
            boundary_instance_id: attempt.boundary_instance_id().to_owned(),
            attempt_id: attempt.attempt_id().to_owned(),
            operation_id: operation_id.into(),
            native_replay_identity: native_replay_identity.into(),
            action_key_digest: action_key.digest().to_owned(),
            authorization_commitment: authorization_commitment.into(),
            authorizer_identity: authorizer_identity.into(),
        };
        if out.operation_id.trim().is_empty()
            || out.native_replay_identity.trim().is_empty()
            || out.authorization_commitment.trim().is_empty()
            || out.authorizer_identity.trim().is_empty()
        {
            return Err("recovery authorization fields must be non-empty".into());
        }
        let reconstructed = AttemptIdentityV1::new(
            &out.boundary_kind,
            &out.boundary_instance_id,
            &out.attempt_id,
        )?;
        if reconstructed.digest() != out.attempt_identity
            || out.action_key_digest != action_key.digest()
        {
            return Err("recovery authorization scope is internally inconsistent".into());
        }
        Ok(out)
    }
}

pub trait RecoveryAuthorizer {
    fn verify(
        &self,
        authorization: &PreEntryRecoveryAuthorizationV1,
        attempt: &AttemptRecordV1,
        action_key: &ActionKeyV1,
    ) -> Result<(), String>;
}

/// Exact authorization for abandoning a stranded provider-entry claim.
///
/// This is deliberately distinct from pre-entry recovery: the attempt already
/// reached DISPATCH_PENDING, so abandonment can only move it to INDETERMINATE.
/// Deployment policy must ensure the original claimant has been fenced before
/// this authorization is issued; there is no unsafe timeout-based lease.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ProviderEntryClaimRecoveryAuthorizationV1 {
    pub attempt_identity: String,
    pub boundary_kind: String,
    pub boundary_instance_id: String,
    pub attempt_id: String,
    pub operation_id: String,
    pub native_replay_identity: String,
    pub action_key_digest: String,
    pub claim_token_digest: String,
    pub authorization_commitment: String,
    pub authorizer_identity: String,
}

impl ProviderEntryClaimRecoveryAuthorizationV1 {
    pub fn new(
        attempt: &AttemptIdentityV1,
        operation_id: impl Into<String>,
        native_replay_identity: impl Into<String>,
        action_key: &ActionKeyV1,
        claim_token_digest: impl Into<String>,
        authorization_commitment: impl Into<String>,
        authorizer_identity: impl Into<String>,
    ) -> Result<Self, String> {
        let out = Self {
            attempt_identity: attempt.digest().to_owned(),
            boundary_kind: attempt.boundary_kind().to_owned(),
            boundary_instance_id: attempt.boundary_instance_id().to_owned(),
            attempt_id: attempt.attempt_id().to_owned(),
            operation_id: operation_id.into(),
            native_replay_identity: native_replay_identity.into(),
            action_key_digest: action_key.digest().to_owned(),
            claim_token_digest: claim_token_digest.into(),
            authorization_commitment: authorization_commitment.into(),
            authorizer_identity: authorizer_identity.into(),
        };
        if out.operation_id.trim().is_empty()
            || out.native_replay_identity.trim().is_empty()
            || out.claim_token_digest.trim().is_empty()
            || out.authorization_commitment.trim().is_empty()
            || out.authorizer_identity.trim().is_empty()
        {
            return Err("provider-entry claim recovery fields must be non-empty".into());
        }
        let reconstructed = AttemptIdentityV1::new(
            &out.boundary_kind,
            &out.boundary_instance_id,
            &out.attempt_id,
        )?;
        if reconstructed.digest() != out.attempt_identity
            || out.action_key_digest != action_key.digest()
        {
            return Err("provider-entry claim recovery scope is internally inconsistent".into());
        }
        Ok(out)
    }
}

pub trait ProviderEntryClaimRecoveryAuthorizer {
    fn verify(
        &self,
        authorization: &ProviderEntryClaimRecoveryAuthorizationV1,
        attempt: &AttemptRecordV1,
        action_key: &ActionKeyV1,
        claim: &ProviderEntryClaimV1,
    ) -> Result<(), String>;
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BoundaryOutcome {
    Admitted(AtomicAdmissionDecision),
    ExecutedConfirmed,
    FailedConfirmed,
    IndeterminateHeld { reason: String },
    RecoveryReleasedNotEntered,
    RecoveryHeld { reason: String },
    TerminalAlreadyReached(AttemptRecordState),
    PreEntryStopRequired,
    FinalEntryRejectedNotEntered { reason: String },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum BoundaryError {
    Store(String),
    Semantic(String),
    Mutation(ActionFenceMutationError),
}

impl std::fmt::Display for BoundaryError {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Store(e) => write!(f, "durable store error: {e}"),
            Self::Semantic(e) => write!(f, "boundary semantic error: {e}"),
            Self::Mutation(e) => write!(f, "durable mutation error: {e:?}"),
        }
    }
}

impl std::error::Error for BoundaryError {}

#[derive(Debug)]
pub struct EffectBoundaryHostV1 {
    store: SqliteActionFenceStore,
    entry_claim_nonce: u64,
}

impl EffectBoundaryHostV1 {
    pub fn new(store: SqliteActionFenceStore) -> Result<Self, BoundaryError> {
        store.audit_integrity().map_err(BoundaryError::Store)?;
        Ok(Self {
            store,
            entry_claim_nonce: 0,
        })
    }

    pub fn store(&self) -> &SqliteActionFenceStore {
        &self.store
    }

    pub fn store_mut(&mut self) -> &mut SqliteActionFenceStore {
        &mut self.store
    }

    pub fn admit(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        record: AttemptRecordV1,
    ) -> Result<BoundaryOutcome, BoundaryError> {
        self.store
            .atomically_admit(action_key, attempt_identity, record)
            .map(BoundaryOutcome::Admitted)
            .map_err(BoundaryError::Semantic)
    }

    fn owned_attempt(
        &self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
    ) -> Result<AttemptRecordV1, BoundaryError> {
        let current = self
            .store
            .durably_read_attempt(attempt_identity)
            .map_err(BoundaryError::Store)?
            .ok_or_else(|| BoundaryError::Semantic("attempt does not exist".into()))?;

        if current.attempt_identity != attempt_identity.digest()
            || current.action_key_digest != action_key.digest()
        {
            return Err(BoundaryError::Semantic(
                "durable attempt/action-key identity mismatch".into(),
            ));
        }
        if current.ownership_token_digest != owner_token_digest {
            return Err(BoundaryError::Mutation(
                ActionFenceMutationError::OwnershipTokenMismatch,
            ));
        }
        Ok(current)
    }

    /// Provider invocation is reachable only after a durable read confirms
    /// DISPATCH_PENDING for the exact attempt owner.
    pub fn dispatch<
        P: ProviderAdapter,
        V: OutcomeVerifier,
        G: FinalProviderEntryVerifier,
    >(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        provider: &mut P,
        verifier: &V,
        entry_verifier: &G,
    ) -> Result<BoundaryOutcome, BoundaryError> {
        let current = self.owned_attempt(action_key, attempt_identity, owner_token_digest)?;
        if current.state.is_terminal() {
            return Ok(BoundaryOutcome::TerminalAlreadyReached(current.state));
        }
        if !matches!(current.state, AttemptRecordState::Consumed | AttemptRecordState::Reserved) {
            return Ok(BoundaryOutcome::PreEntryStopRequired);
        }

        self.store
            .atomically_mark_dispatch_pending(action_key, attempt_identity, owner_token_digest)
            .map_err(BoundaryError::Mutation)?;

        let pending = self.owned_attempt(action_key, attempt_identity, owner_token_digest)?;
        if pending.state != AttemptRecordState::DispatchPending {
            return Err(BoundaryError::Semantic(
                "DISPATCH_PENDING was not durably confirmed".into(),
            ));
        }

        self.entry_claim_nonce = self.entry_claim_nonce.wrapping_add(1);
        let claim_token = derive_entry_claim_token(
            attempt_identity,
            owner_token_digest,
            self.entry_claim_nonce,
        );
        let claim = self
            .store
            .atomically_claim_provider_entry(
                action_key,
                attempt_identity,
                owner_token_digest,
                &claim_token,
            )
            .map_err(BoundaryError::Mutation)?;

        let verifier_now_unix_ms = current_unix_ms().map_err(BoundaryError::Store)?;
        let context = ProviderActionContextV1::from_attempt(&pending, action_key)?;
        let final_entry_proof = match entry_verifier.verify(
            &pending,
            &context,
            verifier_now_unix_ms,
        ) {
            Ok(proof) => proof,
            Err(error) => {
                let marker =
                    "final provider-entry proof was stale or scope-mismatched".to_owned();
                match self.store.atomically_release_provider_entry_claim_not_entered(
                    action_key,
                    attempt_identity,
                    owner_token_digest,
                    &claim_token,
                    marker.clone(),
                ) {
                    Ok(()) => return Ok(BoundaryOutcome::FinalEntryRejectedNotEntered {
                        reason: marker,
                    }),
                    Err(release_error) => {
                        return Ok(BoundaryOutcome::IndeterminateHeld {
                            reason: format!(
                                "final-entry gate rejected dispatch and claim release was not confirmed: {release_error:?}"
                            ),
                        });
                    }
                }
            }
            Err(error) => {
                let marker = format!("final provider-entry gate rejected: {error}");
                match self.store.atomically_release_provider_entry_claim_not_entered(
                    action_key,
                    attempt_identity,
                    owner_token_digest,
                    &claim_token,
                    marker.clone(),
                ) {
                    Ok(()) => return Ok(BoundaryOutcome::FinalEntryRejectedNotEntered {
                        reason: marker,
                    }),
                    Err(release_error) => {
                        return Ok(BoundaryOutcome::IndeterminateHeld {
                            reason: format!(
                                "final-entry gate rejected dispatch and claim release was not confirmed: {release_error:?}"
                            ),
                        });
                    }
                }
            }
        };

        let pre_entry_now_unix_ms = current_unix_ms().map_err(BoundaryError::Store)?;
        if !final_entry_proof.matches(&pending, &context)
            || !final_entry_proof.is_fresh(pre_entry_now_unix_ms)
        {
            let marker = "final provider-entry proof expired before provider entry".to_owned();
            match self.store.atomically_release_provider_entry_claim_not_entered(
                action_key,
                attempt_identity,
                owner_token_digest,
                &claim_token,
                marker.clone(),
            ) {
                Ok(()) => return Ok(BoundaryOutcome::FinalEntryRejectedNotEntered {
                    reason: marker,
                }),
                Err(release_error) => {
                    return Ok(BoundaryOutcome::IndeterminateHeld {
                        reason: format!(
                            "final-entry proof expired and claim release was not confirmed: {release_error:?}"
                        ),
                    });
                }
            }
        }

        let permit = ProviderEntryPermitV1::new(
            pending,
            action_key,
            claim,
            final_entry_proof,
            pre_entry_now_unix_ms,
        )?;
        let observation = match provider.invoke(&permit) {
            Ok(value) => value,
            Err(error) => {
                self.store
                    .atomically_mark_invoked(
                        action_key,
                        attempt_identity,
                        owner_token_digest,
                        permit.claim_token_digest(),
                    )
                    .map_err(BoundaryError::Mutation)?;
                return self.mark_indeterminate(
                    action_key,
                    attempt_identity,
                    owner_token_digest,
                    format!("provider invocation outcome is ambiguous: {error}"),
                );
            }
        };

        self.store
            .atomically_mark_invoked(
                action_key,
                attempt_identity,
                owner_token_digest,
                permit.claim_token_digest(),
            )
            .map_err(BoundaryError::Mutation)?;
        let invoked = self.owned_attempt(action_key, attempt_identity, owner_token_digest)?;

        match observation {
            ProviderObservation::Indeterminate { .. } => self.mark_indeterminate(
                action_key,
                attempt_identity,
                owner_token_digest,
                "provider outcome is indeterminate".into(),
            ),
            ProviderObservation::Executed { .. } | ProviderObservation::Failed { .. } => {
                match verifier.verify(
                    &invoked,
                    permit.provider_idempotency_key(),
                    &observation,
                    VerificationPurpose::InitialInvocation,
                ) {
                    Ok(verified) => self.finalize_verified(
                        action_key,
                        attempt_identity,
                        owner_token_digest,
                        &invoked,
                        permit.provider_idempotency_key(),
                        VerificationPurpose::InitialInvocation,
                        verified,
                    ),
                    Err(error) => self.mark_indeterminate(
                        action_key,
                        attempt_identity,
                        owner_token_digest,
                        format!("provider result was not authenticated: {error}"),
                    ),
                }
            }
        }
    }

    fn finalize_verified(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        attempt: &AttemptRecordV1,
        provider_idempotency_key: &str,
        purpose: VerificationPurpose,
        verified: VerifiedTerminalOutcomeV1,
    ) -> Result<BoundaryOutcome, BoundaryError> {
        if !verified.matches(attempt, action_key, purpose, provider_idempotency_key) {
            return Err(BoundaryError::Semantic(
                "terminal verification proof is not bound to the exact attempt, action, or purpose"
                    .into(),
            ));
        }

        let evidence = TerminalEvidenceV1::from_attempt(
            action_key,
            attempt,
            verified.outcome,
            provider_idempotency_key,
            verified.evidence_commitment,
            verified.verifier_identity,
        )
        .map_err(BoundaryError::Semantic)?;

        let terminal_token = match attempt.state {
            AttemptRecordState::Indeterminate => attempt
                .reconciliation_token_digest
                .as_deref()
                .ok_or_else(|| {
                    BoundaryError::Semantic(
                        "Indeterminate attempt lacks reconciliation token".into(),
                    )
                })?,
            AttemptRecordState::Invoked => owner_token_digest,
            _ => {
                return Err(BoundaryError::Semantic(
                    "terminalization requires Invoked or Indeterminate attempt".into(),
                ))
            }
        };

        match verified.outcome {
            TerminalOutcomeV1::Executed => self.store.atomically_close_executed(
                action_key,
                attempt_identity,
                terminal_token,
                &evidence,
            ),
            TerminalOutcomeV1::Failed => self.store.atomically_release_after_failed(
                action_key,
                attempt_identity,
                terminal_token,
                &evidence,
            ),
        }
        .map_err(BoundaryError::Mutation)?;

        let confirmed = self
            .store
            .durably_read_attempt(attempt_identity)
            .map_err(BoundaryError::Store)?
            .ok_or_else(|| BoundaryError::Store("terminal attempt disappeared".into()))?;
        let expected = match verified.outcome {
            TerminalOutcomeV1::Executed => AttemptRecordState::Executed,
            TerminalOutcomeV1::Failed => AttemptRecordState::Failed,
        };
        if confirmed.state != expected
            || confirmed.terminal_evidence_digest.as_deref() != Some(evidence.digest())
        {
            return Ok(BoundaryOutcome::IndeterminateHeld {
                reason: "terminal mutation was not confirmed by exact durable read".into(),
            });
        }

        if verified.outcome == TerminalOutcomeV1::Executed {
            let fence = self
                .store
                .durably_read_fence(action_key)
                .map_err(BoundaryError::Store)?
                .ok_or_else(|| BoundaryError::Store("executed attempt lost its closed fence".into()))?;
            if fence.state != ActionFenceState::Closed
                || fence.owner_attempt_identity != attempt_identity.digest()
                || fence.owner_token_digest != owner_token_digest
            {
                return Ok(BoundaryOutcome::IndeterminateHeld {
                    reason: "executed outcome did not confirm the exact closed fence".into(),
                });
            }
            return Ok(BoundaryOutcome::ExecutedConfirmed);
        }

        if self
            .store
            .durably_read_fence(action_key)
            .map_err(BoundaryError::Store)?
            .is_some()
        {
            return Ok(BoundaryOutcome::IndeterminateHeld {
                reason: "failed outcome still exposes an action fence".into(),
            });
        }

        Ok(BoundaryOutcome::FailedConfirmed)
    }

    fn mark_indeterminate(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        reason: String,
    ) -> Result<BoundaryOutcome, BoundaryError> {
        self.store
            .atomically_mark_indeterminate(action_key, attempt_identity, owner_token_digest)
            .map_err(BoundaryError::Mutation)?;
        Ok(BoundaryOutcome::IndeterminateHeld { reason })
    }

    /// Reconciliation first makes any stranded DISPATCH_PENDING/INVOKED attempt
    /// explicitly INDETERMINATE, preventing a concurrent original path from
    /// recording its outcome after reconciliation begins.
    pub fn reconcile<P: ProviderAdapter, V: OutcomeVerifier>(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        provider: &mut P,
        verifier: &V,
    ) -> Result<BoundaryOutcome, BoundaryError> {
        let current = self.owned_attempt(action_key, attempt_identity, owner_token_digest)?;
        if current.state.is_terminal() {
            return Ok(BoundaryOutcome::TerminalAlreadyReached(current.state));
        }
        if matches!(current.state, AttemptRecordState::Consumed | AttemptRecordState::Reserved) {
            return Ok(BoundaryOutcome::PreEntryStopRequired);
        }
        if current.state == AttemptRecordState::DispatchPending {
            if self
                .store
                .durably_read_provider_entry_claim(attempt_identity)
                .map_err(BoundaryError::Store)?
                .is_some()
            {
                return Ok(BoundaryOutcome::IndeterminateHeld {
                    reason: "provider entry claim is active; reconciliation cannot race provider entry"
                        .into(),
                });
            }

            self.store
                .atomically_mark_indeterminate(action_key, attempt_identity, owner_token_digest)
                .map_err(BoundaryError::Mutation)?;
        }

        let indeterminate = self.owned_attempt(action_key, attempt_identity, owner_token_digest)?;
        let provider_context =
            ProviderActionContextV1::from_attempt(&indeterminate, action_key)?;
        let observation = match provider.reconcile(&provider_context) {
            Ok(value) => value,
            Err(error) => {
                return Ok(BoundaryOutcome::IndeterminateHeld {
                    reason: format!("authoritative reconciliation unavailable: {error}"),
                });
            }
        };

        match observation {
            ProviderObservation::Indeterminate { .. } => Ok(BoundaryOutcome::IndeterminateHeld {
                reason: "authoritative reconciliation remains indeterminate".into(),
            }),
            ProviderObservation::Executed { .. } | ProviderObservation::Failed { .. } => {
                let provider_idempotency_key =
                    indeterminate.provider_idempotency_key.clone();
                match verifier.verify(
                    &indeterminate,
                    &provider_idempotency_key,
                    &observation,
                    VerificationPurpose::Reconciliation,
                ) {
                    Ok(verified) => self.finalize_verified(
                        action_key,
                        attempt_identity,
                        owner_token_digest,
                        &indeterminate,
                        &provider_idempotency_key,
                        VerificationPurpose::Reconciliation,
                        verified,
                    ),
                    Err(error) => Ok(BoundaryOutcome::IndeterminateHeld {
                        reason: format!("reconciliation evidence was not authenticated: {error}"),
                    }),
                }
            }
        }
    }

    /// Recover a stranded provider-entry claim into INDETERMINATE.
    ///
    /// The authorization is exact-attempt scoped and must be issued only after
    /// the deployment has fenced the old claimant. The durable mutation itself
    /// is atomic, so the claim cannot be cleared while leaving the attempt
    /// ambiguously pre-entry.
    pub fn recover_provider_entry_claim<R: ProviderEntryClaimRecoveryAuthorizer>(
        &mut self,
        action_key: &ActionKeyV1,
        authorization: &ProviderEntryClaimRecoveryAuthorizationV1,
        authorizer: &R,
    ) -> Result<BoundaryOutcome, BoundaryError> {
        let attempt_identity = AttemptIdentityV1::new(
            &authorization.boundary_kind,
            &authorization.boundary_instance_id,
            &authorization.attempt_id,
        )
        .map_err(BoundaryError::Semantic)?;

        if attempt_identity.digest() != authorization.attempt_identity {
            return Err(BoundaryError::Semantic(
                "provider-entry claim recovery attempt digest mismatch".into(),
            ));
        }

        let current = self
            .store
            .durably_read_attempt(&attempt_identity)
            .map_err(BoundaryError::Store)?
            .ok_or_else(|| {
                BoundaryError::Semantic("claim recovery target attempt does not exist".into())
            })?;

        let claim = self
            .store
            .durably_read_provider_entry_claim(&attempt_identity)
            .map_err(BoundaryError::Store)?
            .ok_or_else(|| {
                BoundaryError::Semantic("claim recovery target has no active provider-entry claim".into())
            })?;

        if current.attempt_identity != attempt_identity.digest()
            || current.action_key_digest != action_key.digest()
            || current.operation_id != authorization.operation_id
            || current.native_replay_identity != authorization.native_replay_identity
            || current.state != AttemptRecordState::DispatchPending
            || claim.claim_token_digest != authorization.claim_token_digest
            || claim.attempt_identity != current.attempt_identity
            || claim.action_key_digest != current.action_key_digest
            || claim.owner_token_digest != current.ownership_token_digest
        {
            return Err(BoundaryError::Semantic(
                "provider-entry claim recovery authorization does not exactly bind the claim"
                    .into(),
            ));
        }

        authorizer
            .verify(authorization, &current, action_key, &claim)
            .map_err(BoundaryError::Semantic)?;

        self.store
            .atomically_recover_provider_entry_claim(
                action_key,
                &attempt_identity,
                &current.ownership_token_digest,
                &authorization.claim_token_digest,
            )
            .map_err(BoundaryError::Mutation)?;

        let confirmed = self
            .store
            .durably_read_attempt(&attempt_identity)
            .map_err(BoundaryError::Store)?
            .ok_or_else(|| {
                BoundaryError::Store("recovered attempt disappeared after claim recovery".into())
            })?;
        if confirmed.state != AttemptRecordState::Indeterminate
            || self
                .store
                .durably_read_provider_entry_claim(&attempt_identity)
                .map_err(BoundaryError::Store)?
                .is_some()
        {
            return Ok(BoundaryOutcome::RecoveryHeld {
                reason: "claim recovery was not confirmed by exact durable reads".into(),
            });
        }

        Ok(BoundaryOutcome::IndeterminateHeld {
            reason: "stranded provider-entry claim was explicitly recovered; reconciliation required"
                .into(),
        })
    }

    /// Pre-entry recovery is a distinct operation and is impossible once the
    /// attempt reached DISPATCH_PENDING.
    pub fn recover_pre_entry<R: RecoveryAuthorizer>(
        &mut self,
        action_key: &ActionKeyV1,
        authorization: &PreEntryRecoveryAuthorizationV1,
        marker: impl Into<String>,
        authorizer: &R,
    ) -> Result<BoundaryOutcome, BoundaryError> {
        let attempt_identity = AttemptIdentityV1::new(
            &authorization.boundary_kind,
            &authorization.boundary_instance_id,
            &authorization.attempt_id,
        )
        .map_err(BoundaryError::Semantic)?;

        if attempt_identity.digest() != authorization.attempt_identity {
            return Err(BoundaryError::Semantic(
                "recovery attempt digest mismatch".into(),
            ));
        }

        let current = self
            .store
            .durably_read_attempt(&attempt_identity)
            .map_err(BoundaryError::Store)?
            .ok_or_else(|| BoundaryError::Semantic("recovery target attempt does not exist".into()))?;

        if current.attempt_identity != attempt_identity.digest()
            || current.action_key_digest != action_key.digest()
            || current.operation_id != authorization.operation_id
            || current.native_replay_identity != authorization.native_replay_identity
        {
            return Err(BoundaryError::Semantic(
                "recovery authorization does not exactly bind the attempt".into(),
            ));
        }

        authorizer
            .verify(authorization, &current, action_key)
            .map_err(BoundaryError::Semantic)?;

        if !matches!(current.state, AttemptRecordState::Consumed | AttemptRecordState::Reserved) {
            return Ok(BoundaryOutcome::RecoveryHeld {
                reason: "attempt is no longer provably pre-entry".into(),
            });
        }

        let marker = marker.into();
        self.store
            .atomically_release_not_entered(
                action_key,
                &attempt_identity,
                &current.ownership_token_digest,
                marker.clone(),
            )
            .map_err(BoundaryError::Mutation)?;

        let confirmed = self
            .store
            .durably_read_attempt(&attempt_identity)
            .map_err(BoundaryError::Store)?
            .ok_or_else(|| BoundaryError::Store("recovery attempt disappeared".into()))?;

        if confirmed.state == AttemptRecordState::NotEntered
            && confirmed.not_entered_marker.as_deref() == Some(marker.as_str())
        {
            return Ok(BoundaryOutcome::RecoveryReleasedNotEntered);
        }

        Ok(BoundaryOutcome::RecoveryHeld {
            reason: "pre-entry recovery was not confirmed by exact durable read".into(),
        })
    }
}


#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::{Arc, Mutex};
    use tempfile::tempdir;

    struct FakeProvider {
        invocation: ProviderObservation,
        reconciliation: ProviderObservation,
        invoked_states: Arc<Mutex<Vec<AttemptRecordState>>>,
    }

    impl ProviderAdapter for FakeProvider {
        fn invoke(&mut self, permit: &ProviderEntryPermitV1) -> Result<ProviderObservation, String> {
            assert!(permit
                .provider_idempotency_key()
                .starts_with("constitutional-provider-idempotency-v1:"));
            assert!(!permit.context().operation_id().is_empty());
            self.invoked_states
                .lock()
                .unwrap()
                .push(AttemptRecordState::DispatchPending);
            Ok(self.invocation.clone())
        }

        fn reconcile(&mut self, context: &ProviderActionContextV1) -> Result<ProviderObservation, String> {
            assert!(!context.operation_id().is_empty());
            assert!(!context.native_replay_identity().is_empty());
            Ok(self.reconciliation.clone())
        }
    }

    struct AllowFinalEntry;

    impl FinalProviderEntryVerifier for AllowFinalEntry {
        fn verify(
            &self,
            attempt: &AttemptRecordV1,
            context: &ProviderActionContextV1,
            now_unix_ms: u64,
        ) -> Result<FinalProviderEntryProofV1, String> {
            FinalProviderEntryProofV1::new(
                attempt,
                context,
                now_unix_ms,
                now_unix_ms.saturating_add(60_000),
                "authorization-snapshot-v1",
                "status-snapshot-v1",
                "final-entry-verifier-v1",
            )
        }
    }

    struct ExpiredFinalEntry;

    impl FinalProviderEntryVerifier for ExpiredFinalEntry {
        fn verify(
            &self,
            attempt: &AttemptRecordV1,
            context: &ProviderActionContextV1,
            now_unix_ms: u64,
        ) -> Result<FinalProviderEntryProofV1, String> {
            FinalProviderEntryProofV1::new(
                attempt,
                context,
                now_unix_ms.saturating_sub(60_000),
                now_unix_ms.saturating_sub(1),
                "authorization-snapshot-v1",
                "status-snapshot-v1",
                "expired-entry-verifier-v1",
            )
        }
    }

    struct RejectFinalEntry;

    impl FinalProviderEntryVerifier for RejectFinalEntry {
        fn verify(
            &self,
            _attempt: &AttemptRecordV1,
            _context: &ProviderActionContextV1,
            _now_unix_ms: u64,
        ) -> Result<FinalProviderEntryProofV1, String> {
            Err("authorization/status is stale".into())
        }
    }

    struct Verifier;

    impl OutcomeVerifier for Verifier {
        fn verify(
            &self,
            _attempt: &AttemptRecordV1,
            _provider_idempotency_key: &str,
            observation: &ProviderObservation,
            _purpose: VerificationPurpose,
        ) -> Result<VerifiedTerminalOutcomeV1, String> {
            match observation {
                ProviderObservation::Executed { evidence_commitment } =>
                    VerifiedTerminalOutcomeV1::new(
                        _attempt,
                        _purpose,
                        _provider_idempotency_key,
                        TerminalOutcomeV1::Executed,
                        evidence_commitment.clone(),
                        "verified-provider-v1",
                    ),
                ProviderObservation::Failed { evidence_commitment } =>
                    VerifiedTerminalOutcomeV1::new(
                        _attempt,
                        _purpose,
                        _provider_idempotency_key,
                        TerminalOutcomeV1::Failed,
                        evidence_commitment.clone(),
                        "verified-provider-v1",
                    ),
                ProviderObservation::Indeterminate { .. } => {
                    Err("indeterminate observation is not terminal".into())
                }
            }
        }
    }

    struct AllowRecovery;

    struct AllowClaimRecovery;
    impl ProviderEntryClaimRecoveryAuthorizer for AllowClaimRecovery {
        fn verify(
            &self,
            authorization: &ProviderEntryClaimRecoveryAuthorizationV1,
            _attempt: &AttemptRecordV1,
            _action_key: &ActionKeyV1,
            claim: &ProviderEntryClaimV1,
        ) -> Result<(), String> {
            if authorization.authorization_commitment != "authorized-claim-recovery-v1" {
                return Err("claim recovery authorization rejected".into());
            }
            if authorization.claim_token_digest != claim.claim_token_digest {
                return Err("claim recovery claim-token mismatch".into());
            }
            Ok(())
        }
    }

    impl RecoveryAuthorizer for AllowRecovery {
        fn verify(
            &self,
            authorization: &PreEntryRecoveryAuthorizationV1,
            _attempt: &AttemptRecordV1,
            _action_key: &ActionKeyV1,
        ) -> Result<(), String> {
            if authorization.authorization_commitment == "authorized-recovery-v1" {
                Ok(())
            } else {
                Err("recovery authorization rejected".into())
            }
        }
    }

    fn action() -> ActionKeyV1 {
        ActionKeyV1::new("rp-test", "provider-target", "material-action-1").unwrap()
    }

    fn identity(id: &str) -> AttemptIdentityV1 {
        AttemptIdentityV1::new("governance", "host-1", id).unwrap()
    }

    fn attempt_record(id: &str, op: &str, state: AttemptRecordState) -> AttemptRecordV1 {
        let action = action();
        AttemptRecordV1::new(
            &identity(id),
            op,
            format!("native-{op}"),
            action.material_action_digest(),
            &action,
            Some("provider-seed".into()),
            Some("provider-descriptor".into()),
            "provider-env",
            "provider-audience",
            "provider-adapter-v1",
            format!("owner-{id}"),
            state,
        )
        .unwrap()
    }

    #[test]
    fn provider_idempotency_key_excludes_operation_identifier() {
        let action_key = action();
        let attempt_identity_a = identity("attempt-idempotency-a");
        let mut first = attempt_record(
            "attempt-idempotency-a",
            "operation-one",
            AttemptRecordState::DispatchPending,
        );
        first.native_replay_identity = "native-replay-stable".into();
        first.operation_id = "operation-one".into();
        first.validate().unwrap();

        let mut second = first.clone();
        second.operation_id = "operation-two".into();
        second.validate().unwrap();

        let first_key = first.provider_idempotency_key.clone();
        let second_key = second.provider_idempotency_key.clone();
        assert_eq!(first_key, second_key);

        let mut different_replay = second.clone();
        different_replay.native_replay_identity = "native-replay-different".into();
        different_replay.validate().unwrap();

        assert_ne!(second_key, different_replay.provider_idempotency_key);
        assert_eq!(first.attempt_identity, attempt_identity_a.digest());
    }

    #[test]
    fn provider_is_not_called_before_confirmed_dispatch_pending() {
        let dir = tempdir().unwrap();
        let store = SqliteActionFenceStore::open(dir.path().join("dispatch.db")).unwrap();
        let mut boundary = EffectBoundaryHostV1::new(store).unwrap();
        let action = action();
        let owner = identity("attempt-1");
        boundary
            .admit(
                &action,
                &owner,
                attempt_record("attempt-1", "operation-1", AttemptRecordState::Consumed),
            )
            .unwrap();

        let states = Arc::new(Mutex::new(Vec::new()));
        let mut provider = FakeProvider {
            invocation: ProviderObservation::Executed {
                evidence_commitment: "exec-evidence".into(),
            },
            reconciliation: ProviderObservation::Indeterminate {
                evidence_commitment: None,
            },
            invoked_states: Arc::clone(&states),
        };

        assert_eq!(
            boundary
                .dispatch(
                    &action,
                    &owner,
                    "owner-attempt-1",
                    &mut provider,
                    &Verifier,
                    &AllowFinalEntry,
                )
                .unwrap(),
            BoundaryOutcome::ExecutedConfirmed
        );
        assert_eq!(&*states.lock().unwrap(), &[AttemptRecordState::DispatchPending]);
    }

    #[test]
    fn invocation_error_holds_fence_as_indeterminate() {
        struct ErrorProvider;
        impl ProviderAdapter for ErrorProvider {
            fn invoke(&mut self, _permit: &ProviderEntryPermitV1) -> Result<ProviderObservation, String> {
                Err("timeout after send".into())
            }
            fn reconcile(&mut self, _context: &ProviderActionContextV1) -> Result<ProviderObservation, String> {
                unreachable!()
            }
        }

        let dir = tempdir().unwrap();
        let store = SqliteActionFenceStore::open(dir.path().join("ambiguous.db")).unwrap();
        let mut boundary = EffectBoundaryHostV1::new(store).unwrap();
        let action = action();
        let owner = identity("attempt-ambiguous");
        boundary
            .admit(
                &action,
                &owner,
                attempt_record("attempt-ambiguous", "operation-ambiguous", AttemptRecordState::Consumed),
            )
            .unwrap();

        let mut provider = ErrorProvider;
        assert!(matches!(
            boundary
                .dispatch(&action, &owner, "owner-attempt-ambiguous", &mut provider, &Verifier, &AllowFinalEntry)
                .unwrap(),
            BoundaryOutcome::IndeterminateHeld { .. }
        ));
        assert_eq!(
            boundary
                .store()
                .durably_read_attempt(&owner)
                .unwrap()
                .unwrap()
                .state,
            AttemptRecordState::Indeterminate
        );
    }

    struct MismatchedIdempotencyVerifier;

    impl OutcomeVerifier for MismatchedIdempotencyVerifier {
        fn verify(
            &self,
            attempt: &AttemptRecordV1,
            _provider_idempotency_key: &str,
            _observation: &ProviderObservation,
            purpose: VerificationPurpose,
        ) -> Result<VerifiedTerminalOutcomeV1, String> {
            VerifiedTerminalOutcomeV1::new(
                attempt,
                purpose,
                "constitutional-provider-idempotency-v1:wrong-key",
                TerminalOutcomeV1::Executed,
                "wrong-idempotency-proof",
                "malbound-verifier",
            )
        }
    }

    struct MismatchedVerifier;

    impl OutcomeVerifier for MismatchedVerifier {
        fn verify(
            &self,
            attempt: &AttemptRecordV1,
            _provider_idempotency_key: &str,
            _observation: &ProviderObservation,
            _purpose: VerificationPurpose,
        ) -> Result<VerifiedTerminalOutcomeV1, String> {
            let wrong_identity = AttemptIdentityV1::new(
                "governance",
                "host-1",
                "different-attempt",
            )?;
            let wrong_attempt = AttemptRecordV1::new(
                &wrong_identity,
                &attempt.operation_id,
                &attempt.native_replay_identity,
                &attempt.action_digest,
                &action(),
                attempt.provider_reference_seed_digest.clone(),
                attempt.provider_reference_descriptor_digest.clone(),
                attempt.provider_environment.clone(),
                attempt.provider_audience.clone(),
                attempt.adapter_identity.clone(),
                attempt.ownership_token_digest.clone(),
                AttemptRecordState::Invoked,
            )?;
            VerifiedTerminalOutcomeV1::new(
                &wrong_attempt,
                VerificationPurpose::Reconciliation,
                "constitutional-provider-idempotency-v1:mismatched",
                TerminalOutcomeV1::Executed,
                "mismatched-proof",
                "malbound-verifier",
            )
        }
    }

    #[test]
    fn final_entry_gate_rejection_cannot_reach_provider() {
        let dir = tempdir().unwrap();
        let store = SqliteActionFenceStore::open(dir.path().join("final-gate-rejected.db")).unwrap();
        let mut boundary = EffectBoundaryHostV1::new(store).unwrap();
        let action_key = action();
        let owner = identity("attempt-final-gate-rejected");
        boundary
            .admit(
                &action_key,
                &owner,
                attempt_record(
                    "attempt-final-gate-rejected",
                    "operation-final-gate-rejected",
                    AttemptRecordState::Consumed,
                ),
            )
            .unwrap();

        struct MustNotInvoke;
        impl ProviderAdapter for MustNotInvoke {
            fn invoke(
                &mut self,
                _permit: &ProviderEntryPermitV1,
            ) -> Result<ProviderObservation, String> {
                panic!("final-entry rejection must prevent provider invocation");
            }

            fn reconcile(
                &mut self,
                _context: &ProviderActionContextV1,
            ) -> Result<ProviderObservation, String> {
                unreachable!()
            }
        }

        let result = boundary
            .dispatch(
                &action_key,
                &owner,
                "owner-attempt-final-gate-rejected",
                &mut MustNotInvoke,
                &Verifier,
                &RejectFinalEntry,
            )
            .unwrap();

        assert!(matches!(
            result,
            BoundaryOutcome::FinalEntryRejectedNotEntered { reason }
                if reason.contains("authorization/status is stale")
        ));
        let attempt = boundary.store.durably_read_attempt(&owner).unwrap().unwrap();
        assert_eq!(attempt.state, AttemptRecordState::NotEntered);
        assert!(attempt.not_entered_marker.is_some());
        assert!(boundary.store.durably_read_fence(&action_key).unwrap().is_none());
        assert!(boundary
            .store
            .durably_read_provider_entry_claim(&owner)
            .unwrap()
            .is_none());
    }

    #[test]
    fn expired_final_entry_proof_cannot_reach_provider() {
        let dir = tempdir().unwrap();
        let store = SqliteActionFenceStore::open(dir.path().join("final-gate-expired.db")).unwrap();
        let mut boundary = EffectBoundaryHostV1::new(store).unwrap();
        let action_key = action();
        let owner = identity("attempt-final-gate-expired");
        boundary
            .admit(
                &action_key,
                &owner,
                attempt_record(
                    "attempt-final-gate-expired",
                    "operation-final-gate-expired",
                    AttemptRecordState::Consumed,
                ),
            )
            .unwrap();

        struct MustNotInvoke;
        impl ProviderAdapter for MustNotInvoke {
            fn invoke(
                &mut self,
                _permit: &ProviderEntryPermitV1,
            ) -> Result<ProviderObservation, String> {
                panic!("expired final-entry proof must prevent provider invocation");
            }

            fn reconcile(
                &mut self,
                _context: &ProviderActionContextV1,
            ) -> Result<ProviderObservation, String> {
                unreachable!()
            }
        }

        let result = boundary
            .dispatch(
                &action_key,
                &owner,
                "owner-attempt-final-gate-expired",
                &mut MustNotInvoke,
                &Verifier,
                &ExpiredFinalEntry,
            )
            .unwrap();

        assert!(matches!(
            result,
            BoundaryOutcome::FinalEntryRejectedNotEntered { reason }
                if reason.contains("stale or not bound")
        ));
        assert_eq!(
            boundary.store.durably_read_attempt(&owner).unwrap().unwrap().state,
            AttemptRecordState::NotEntered
        );
        assert!(boundary.store.durably_read_fence(&action_key).unwrap().is_none());
        assert!(boundary
            .store
            .durably_read_provider_entry_claim(&owner)
            .unwrap()
            .is_none());
    }

    #[test]
    fn mismatched_provider_idempotency_proof_holds_the_fence() {
        let dir = tempdir().unwrap();
        let store = SqliteActionFenceStore::open(dir.path().join("mismatch-idempotency.db")).unwrap();
        let mut boundary = EffectBoundaryHostV1::new(store).unwrap();
        let action_key = action();
        let owner = identity("attempt-mismatch-idempotency");
        let record = attempt_record(
            "attempt-mismatch-idempotency",
            "operation-mismatch-idempotency",
            AttemptRecordState::Consumed,
        );

        boundary.admit(&action_key, &owner, record).unwrap();

        let mut provider = FakeProvider {
            invocation: ProviderObservation::Executed {
                evidence_commitment: "provider-proof".into(),
            },
            reconciliation: ProviderObservation::Executed {
                evidence_commitment: "reconciled-proof".into(),
            },
            invoked_states: Arc::new(Mutex::new(Vec::new())),
        };

        let result = boundary.dispatch(
            &action_key,
            &owner,
            "owner-attempt-mismatch-idempotency",
            &mut provider,
            &MismatchedIdempotencyVerifier,
            &AllowFinalEntry,
        );
        assert!(matches!(
            result,
            Err(BoundaryError::Semantic(message))
                if message.contains("not bound to the exact attempt, action, or purpose")
        ));

        let attempt = boundary
            .store
            .durably_read_attempt(&owner)
            .unwrap()
            .unwrap();
        assert_eq!(attempt.state, AttemptRecordState::Invoked);
        assert!(boundary
            .store
            .durably_read_fence(&action_key)
            .unwrap()
            .is_some());
    }

    #[test]
    fn mismatched_terminal_verification_proof_holds_the_fence() {
        let dir = tempdir().unwrap();
        let store = SqliteActionFenceStore::open(dir.path().join("mismatch.db")).unwrap();
        let mut boundary = EffectBoundaryHostV1::new(store).unwrap();
        let action_key = action();
        let owner = identity("attempt-mismatch");
        let record = attempt_record("attempt-mismatch", "operation-mismatch", AttemptRecordState::Consumed);

        assert_eq!(
            boundary.admit(&action_key, &owner, record).unwrap(),
            BoundaryOutcome::Admitted(AtomicAdmissionDecision::Admitted)
        );

        let mut provider = FakeProvider {
            invocation: ProviderObservation::Executed {
                evidence_commitment: "provider-proof".into(),
            },
            reconciliation: ProviderObservation::Executed {
                evidence_commitment: "reconciled-proof".into(),
            },
            invoked_states: Arc::new(Mutex::new(Vec::new())),
        };

        let result = boundary.dispatch(
            &action_key,
            &owner,
            "owner-attempt-mismatch",
            &mut provider,
            &MismatchedVerifier,
            &AllowFinalEntry,
        );
        assert!(matches!(
            result,
            Err(BoundaryError::Semantic(message))
                if message.contains("not bound to the exact attempt")
        ));

        let attempt = boundary
            .store
            .durably_read_attempt(&owner)
            .unwrap()
            .unwrap();
        assert_eq!(attempt.state, AttemptRecordState::Invoked);

        let fence = boundary.store.durably_read_fence(&action_key).unwrap();
        assert!(fence.is_some());
        assert_eq!(attempt.reconciliation_token_digest.is_none(), true);
    }

    #[test]
    fn active_provider_entry_claim_blocks_reconciliation() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("claim-race.db");
        let store = SqliteActionFenceStore::open(&path).unwrap();
        let mut boundary = EffectBoundaryHostV1::new(store).unwrap();
        let action_key = action();
        let owner = identity("attempt-claim-race");
        let record = attempt_record(
            "attempt-claim-race",
            "operation-claim-race",
            AttemptRecordState::Consumed,
        );

        boundary.admit(&action_key, &owner, record).unwrap();
        boundary
            .store
            .atomically_mark_dispatch_pending(
                &action_key,
                &owner,
                "owner-attempt-claim-race",
            )
            .unwrap();

        let claim = boundary
            .store
            .atomically_claim_provider_entry(
                &action_key,
                &owner,
                "owner-attempt-claim-race",
                "constitutional-provider-entry-claim-token-v1:test",
            )
            .unwrap();
        assert_eq!(claim.action_key_digest, action_key.digest());

        struct MustNotRun;
        impl ProviderAdapter for MustNotRun {
            fn invoke(
                &mut self,
                _permit: &ProviderEntryPermitV1,
            ) -> Result<ProviderObservation, String> {
                panic!("reconciliation test must not invoke provider");
            }
            fn reconcile(
                &mut self,
                _context: &ProviderActionContextV1,
            ) -> Result<ProviderObservation, String> {
                panic!("active claim must block reconciliation before provider access");
            }
        }

        let result = boundary.reconcile(
            &action_key,
            &owner,
            "owner-attempt-claim-race",
            &mut MustNotRun,
            &Verifier,
        );
        assert!(matches!(
            result,
            Ok(BoundaryOutcome::IndeterminateHeld { reason })
                if reason.contains("provider entry claim is active")
        ));
        assert_eq!(
            boundary
                .store
                .durably_read_attempt(&owner)
                .unwrap()
                .unwrap()
                .state,
            AttemptRecordState::DispatchPending
        );
        assert!(boundary
            .store
            .durably_read_provider_entry_claim(&owner)
            .unwrap()
            .is_some());
    }

    #[test]
    fn stranded_provider_entry_claim_requires_exact_recovery() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("claim-recovery.db");
        let store = SqliteActionFenceStore::open(&path).unwrap();
        let mut boundary = EffectBoundaryHostV1::new(store).unwrap();
        let action_key = action();
        let owner = identity("attempt-claim-recovery");
        let record = attempt_record(
            "attempt-claim-recovery",
            "operation-claim-recovery",
            AttemptRecordState::Consumed,
        );

        boundary.admit(&action_key, &owner, record).unwrap();
        boundary
            .store
            .atomically_mark_dispatch_pending(
                &action_key,
                &owner,
                "owner-attempt-claim-recovery",
            )
            .unwrap();
        let claim = boundary
            .store
            .atomically_claim_provider_entry(
                &action_key,
                &owner,
                "owner-attempt-claim-recovery",
                "constitutional-provider-entry-claim-token-v1:recovery-test",
            )
            .unwrap();

        let authorization = ProviderEntryClaimRecoveryAuthorizationV1::new(
            &owner,
            "operation-claim-recovery",
            "native-operation-claim-recovery",
            &action_key,
            claim.claim_token_digest.clone(),
            "authorized-claim-recovery-v1",
            "recovery-authority-v1",
        )
        .unwrap();

        let result = boundary.recover_provider_entry_claim(
            &action_key,
            &authorization,
            &AllowClaimRecovery,
        )
        .unwrap();

        assert!(matches!(
            result,
            BoundaryOutcome::IndeterminateHeld { reason }
                if reason.contains("stranded provider-entry claim")
        ));
        let attempt = boundary.store.durably_read_attempt(&owner).unwrap().unwrap();
        assert_eq!(attempt.state, AttemptRecordState::Indeterminate);
        assert!(boundary.store.durably_read_provider_entry_claim(&owner).unwrap().is_none());
        assert!(attempt.reconciliation_token_digest.is_some());
    }

    #[test]
    fn pre_entry_recovery_cannot_release_a_dispatch_pending_attempt() {
        let dir = tempdir().unwrap();
        let store = SqliteActionFenceStore::open(dir.path().join("recovery.db")).unwrap();
        let mut boundary = EffectBoundaryHostV1::new(store).unwrap();
        let action = action();
        let owner = identity("attempt-recovery");
        boundary
            .admit(
                &action,
                &owner,
                attempt_record("attempt-recovery", "operation-recovery", AttemptRecordState::Consumed),
            )
            .unwrap();
        boundary
            .store_mut()
            .atomically_mark_dispatch_pending(&action, &owner, "owner-attempt-recovery")
            .unwrap();

        let auth = PreEntryRecoveryAuthorizationV1::new(
            &owner,
            "operation-recovery",
            "native-operation-recovery",
            &action,
            "authorized-recovery-v1",
            "recovery-authorizer-v1",
        )
        .unwrap();

        assert!(matches!(
            boundary
                .recover_pre_entry(
                    &action,
                    &auth,
                    "not-entered-marker",
                    &AllowRecovery,
                )
                .unwrap(),
            BoundaryOutcome::RecoveryHeld { .. }
        ));
        assert_eq!(
            boundary.store().durably_read_fence(&action).unwrap().unwrap().state,
            ActionFenceState::Occupied
        );
    }
}

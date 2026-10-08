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
    TerminalEvidenceV1, TerminalOutcomeV1,
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
    pub outcome: TerminalOutcomeV1,
    pub evidence_commitment: String,
    pub verifier_identity: String,
}

/// Implementations must not cross into the protected provider until the
/// supplied attempt is in durable DISPATCH_PENDING.
pub trait ProviderAdapter {
    fn invoke(&mut self, attempt: &AttemptRecordV1) -> Result<ProviderObservation, String>;
    fn reconcile(&mut self, attempt: &AttemptRecordV1) -> Result<ProviderObservation, String>;
}

/// Authentication/authorization for terminal provider evidence remains outside
/// the durable store.
pub trait OutcomeVerifier {
    fn verify(
        &self,
        attempt: &AttemptRecordV1,
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
}

impl EffectBoundaryHostV1 {
    pub fn new(store: SqliteActionFenceStore) -> Result<Self, BoundaryError> {
        store.audit_integrity().map_err(BoundaryError::Store)?;
        Ok(Self { store })
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
    pub fn dispatch<P: ProviderAdapter, V: OutcomeVerifier>(
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

        let observation = match provider.invoke(&pending) {
            Ok(value) => value,
            Err(error) => {
                return self.hold_indeterminate(
                    action_key,
                    attempt_identity,
                    owner_token_digest,
                    format!("provider invocation outcome is ambiguous: {error}"),
                );
            }
        };

        self.store
            .atomically_mark_invoked(action_key, attempt_identity, owner_token_digest)
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
                    &observation,
                    VerificationPurpose::InitialInvocation,
                ) {
                    Ok(verified) => self.finalize_verified(
                        action_key,
                        attempt_identity,
                        owner_token_digest,
                        &invoked,
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
        verified: VerifiedTerminalOutcomeV1,
    ) -> Result<BoundaryOutcome, BoundaryError> {
        let evidence = TerminalEvidenceV1::from_attempt(
            action_key,
            attempt,
            verified.outcome,
            verified.evidence_commitment,
            verified.verifier_identity,
        )
        .map_err(BoundaryError::Semantic)?;

        match verified.outcome {
            TerminalOutcomeV1::Executed => self.store.atomically_close_executed(
                action_key,
                attempt_identity,
                owner_token_digest,
                &evidence,
            ),
            TerminalOutcomeV1::Failed => self.store.atomically_release_after_failed(
                action_key,
                attempt_identity,
                owner_token_digest,
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

    fn hold_indeterminate(
        &mut self,
        action_key: &ActionKeyV1,
        attempt_identity: &AttemptIdentityV1,
        owner_token_digest: &str,
        reason: String,
    ) -> Result<BoundaryOutcome, BoundaryError> {
        self.store
            .atomically_mark_invoked(action_key, attempt_identity, owner_token_digest)
            .map_err(BoundaryError::Mutation)?;
        self.mark_indeterminate(action_key, attempt_identity, owner_token_digest, reason)
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
        if matches!(
            current.state,
            AttemptRecordState::DispatchPending | AttemptRecordState::Invoked
        ) {
            self.store
                .atomically_mark_indeterminate(action_key, attempt_identity, owner_token_digest)
                .map_err(BoundaryError::Mutation)?;
        }

        let indeterminate = self.owned_attempt(action_key, attempt_identity, owner_token_digest)?;
        let observation = match provider.reconcile(&indeterminate) {
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
                match verifier.verify(
                    &indeterminate,
                    &observation,
                    VerificationPurpose::Reconciliation,
                ) {
                    Ok(verified) => self.finalize_verified(
                        action_key,
                        attempt_identity,
                        owner_token_digest,
                        &indeterminate,
                        verified,
                    ),
                    Err(error) => Ok(BoundaryOutcome::IndeterminateHeld {
                        reason: format!("reconciliation evidence was not authenticated: {error}"),
                    }),
                }
            }
        }
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

//! Guarded public API for MYC-CONST-003D1D-F0.
//!
//! The transition kernel remains private so callers cannot obtain mutable raw
//! records/registries and bypass reconciliation-history checks. All externally
//! visible state transitions pass through this facade.

mod kernel {
    include!("lib.rs");
}

use std::collections::BTreeMap;

pub use kernel::{
    EXECUTION_ID_PREFIX, INTENT_COMMITMENT_PREFIX, MAX_ID_LEN, MAX_REF_LEN,
    REQUEST_COMMITMENT_PREFIX, TREASURY_EFFECT_SCHEMA_VERSION, EffectAttempt,
    EffectIntegrityFault, EffectIntent, EffectState, ObservationDecision, ProviderObservation,
    ProviderObservationKind, RegistryDecision, RegistryIntegrityFault, RetryBasis,
    TreasuryEffectError, TreasuryEffectRequest, TreasuryEffectResult, TreasuryEffectSubject,
};

/// Guarded effect record.
///
/// The underlying transition kernel is intentionally private. Callers can read
/// immutable snapshots and request typed transitions, but cannot mutate state,
/// attempts, observations, or the intent directly.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EffectRecord {
    inner: kernel::EffectRecord,
}

impl EffectRecord {
    pub fn new(intent: EffectIntent) -> TreasuryEffectResult<Self> {
        Ok(Self {
            inner: kernel::EffectRecord::new(intent)?,
        })
    }

    fn from_inner(inner: kernel::EffectRecord) -> Self {
        Self { inner }
    }

    pub fn intent(&self) -> &EffectIntent {
        &self.inner.intent
    }

    pub fn state(&self) -> &EffectState {
        &self.inner.state
    }

    pub fn attempts(&self) -> &[EffectAttempt] {
        &self.inner.attempts
    }

    pub fn observations(&self) -> &BTreeMap<String, ProviderObservation> {
        &self.inner.observations
    }

    pub fn begin_attempt(
        &mut self,
        attempt_id: impl Into<String>,
        started_at_unix_ms: u64,
        retry_basis: RetryBasis,
    ) -> TreasuryEffectResult<EffectAttempt> {
        self.inner
            .begin_attempt(attempt_id, started_at_unix_ms, retry_basis)
    }

    pub fn record_transport_failure(
        &mut self,
        attempt_id: &str,
        transport_error_ref: impl Into<String>,
    ) -> TreasuryEffectResult<()> {
        self.inner
            .record_transport_failure(attempt_id, transport_error_ref)
    }

    pub fn apply_observation(
        &mut self,
        observation: ProviderObservation,
    ) -> TreasuryEffectResult<ObservationDecision> {
        apply_observation_guarded(&mut self.inner, observation)
    }

    pub fn validate_invariants(&self) -> TreasuryEffectResult<()> {
        self.inner.validate_invariants()
    }
}

/// Detect contradictions against the durable observation history before the
/// kernel considers the current state.
///
/// This matters after `KnownNoEffect(attempt N)` has authorized a retry. Once
/// attempt N+1 is in flight, the current state alone no longer remembers that a
/// provider previously asserted no effect through N. A late success receipt for
/// an attempt covered by that assertion is contradictory evidence and must halt
/// rather than silently overwrite the reconciliation history.
fn historical_contradiction(
    record: &kernel::EffectRecord,
    observation: &ProviderObservation,
) -> Option<EffectIntegrityFault> {
    // Preserve the kernel's exact duplicate / observation-ID-collision handling.
    if record.observations.contains_key(&observation.observation_id) {
        return None;
    }

    match &observation.kind {
        ProviderObservationKind::KnownSuccess { .. } => {
            for prior in record.observations.values() {
                if matches!(&prior.kind, ProviderObservationKind::KnownNoEffect { .. })
                    && prior.covers_through_attempt_ordinal
                        >= observation.covers_through_attempt_ordinal
                {
                    return Some(EffectIntegrityFault::SuccessContradictsNoEffect {
                        no_effect_observation_id: prior.observation_id.clone(),
                        success_observation_id: observation.observation_id.clone(),
                    });
                }
            }
        }
        ProviderObservationKind::KnownNoEffect { .. } => {
            for prior in record.observations.values() {
                if matches!(&prior.kind, ProviderObservationKind::KnownSuccess { .. })
                    && prior.covers_through_attempt_ordinal
                        <= observation.covers_through_attempt_ordinal
                {
                    return Some(EffectIntegrityFault::NoEffectContradictsSuccess {
                        success_observation_id: prior.observation_id.clone(),
                        no_effect_observation_id: observation.observation_id.clone(),
                    });
                }
            }
        }
        ProviderObservationKind::UnknownOutcome { .. } => {}
    }

    None
}

fn apply_observation_guarded(
    record: &mut kernel::EffectRecord,
    observation: ProviderObservation,
) -> TreasuryEffectResult<ObservationDecision> {
    observation.validate()?;

    // Identity mismatch remains an ordinary typed rejection; it is not evidence
    // about this record's provider history.
    if observation.execution_id != record.intent.execution_id
        || observation.request_commitment != record.intent.request.request_commitment
    {
        return record.apply_observation(observation);
    }

    if let Some(fault) = historical_contradiction(record, &observation) {
        record.state = EffectState::IntegrityHalted {
            fault: fault.clone(),
        };
        return Ok(ObservationDecision::IntegrityConflict { fault });
    }

    record.apply_observation(observation)
}

/// Guarded registry for deterministic effect identity and one-shot subject use.
///
/// The kernel registry is private and no `record_mut()` equivalent is exposed.
/// Mutating an effect must go through the typed facade transitions below.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TreasuryEffectRegistry {
    inner: kernel::TreasuryEffectRegistry,
}

impl Default for TreasuryEffectRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl TreasuryEffectRegistry {
    pub fn new() -> Self {
        Self {
            inner: kernel::TreasuryEffectRegistry::new(),
        }
    }

    pub fn submit(
        &mut self,
        request: TreasuryEffectRequest,
        committed_at_unix_ms: u64,
    ) -> TreasuryEffectResult<RegistryDecision> {
        self.inner.submit(request, committed_at_unix_ms)
    }

    /// Returns an immutable owned snapshot. Mutating the snapshot cannot mutate
    /// registry truth; all live transitions must use the registry methods below.
    pub fn record(&self, execution_id: &str) -> Option<EffectRecord> {
        self.inner
            .record(execution_id)
            .cloned()
            .map(EffectRecord::from_inner)
    }

    pub fn by_action(&self, action_id: &str) -> Option<EffectRecord> {
        self.inner
            .by_action(action_id)
            .cloned()
            .map(EffectRecord::from_inner)
    }

    pub fn by_authorization(&self, authorization_id: &str) -> Option<EffectRecord> {
        self.inner
            .by_authorization(authorization_id)
            .cloned()
            .map(EffectRecord::from_inner)
    }

    pub fn begin_attempt(
        &mut self,
        execution_id: &str,
        attempt_id: impl Into<String>,
        started_at_unix_ms: u64,
        retry_basis: RetryBasis,
    ) -> TreasuryEffectResult<EffectAttempt> {
        self.inner.record_mut(execution_id)?.begin_attempt(
            attempt_id,
            started_at_unix_ms,
            retry_basis,
        )
    }

    pub fn record_transport_failure(
        &mut self,
        execution_id: &str,
        attempt_id: &str,
        transport_error_ref: impl Into<String>,
    ) -> TreasuryEffectResult<()> {
        self.inner
            .record_mut(execution_id)?
            .record_transport_failure(attempt_id, transport_error_ref)
    }

    pub fn apply_observation(
        &mut self,
        execution_id: &str,
        observation: ProviderObservation,
    ) -> TreasuryEffectResult<ObservationDecision> {
        let record = self.inner.record_mut(execution_id)?;
        apply_observation_guarded(record, observation)
    }

    pub fn validate_invariants(&self) -> TreasuryEffectResult<()> {
        self.inner.validate_invariants()?;

        // The kernel already proves every declared capacity index resolves. The
        // facade additionally rejects an extra capacity index alias that points
        // to a record whose own subject does not declare that exact capacity.
        for (capacity, execution_id) in &self.inner.capacity_index {
            let record = self
                .inner
                .records
                .get(execution_id)
                .ok_or_else(|| TreasuryEffectError::Violation(
                    "capacity index references missing effect record".into(),
                ))?;
            if record.intent.request.subject.capacity_allocation_commitment.as_deref()
                != Some(capacity.as_str())
            {
                return Err(TreasuryEffectError::Violation(
                    "capacity index aliases an execution that does not declare that capacity"
                        .into(),
                ));
            }
        }

        Ok(())
    }
}

#[cfg(test)]
mod facade_tests {
    use super::*;

    fn request() -> TreasuryEffectRequest {
        TreasuryEffectRequest::new(TreasuryEffectSubject {
            operation_id: "op-1".into(),
            action_id: "action-1".into(),
            proposal_id: "proposal-1".into(),
            claim_binding_commitment: "claim-binding-1".into(),
            action_commitment: "action-commitment-1".into(),
            authorization_id: "authorization-1".into(),
            authorization_subject_commitment: "authorization-subject-1".into(),
            treasury_descriptor_commitment: "treasury-descriptor-1".into(),
            allocation_subject_commitment: "allocation-subject-1".into(),
            approval_projection_commitment: "approval-projection-1".into(),
            capacity_allocation_commitment: Some("capacity-allocation-1".into()),
            recipient_did: "did:mycelix:recipient".into(),
            recipient_commitment: "recipient-1".into(),
            value_profile_id: "pending-sap-v1".into(),
            value_authority_commitment: "value-1".into(),
            policy_revision_commitment: "policy-1".into(),
            adapter_profile_id: "adapter-1".into(),
            effect_target_commitment: "target-1".into(),
        })
        .unwrap()
    }

    fn no_effect(record: &EffectRecord, id: &str, through: u32) -> ProviderObservation {
        ProviderObservation {
            observation_id: id.into(),
            execution_id: record.intent().execution_id.clone(),
            request_commitment: record.intent().request.request_commitment.clone(),
            covers_through_attempt_ordinal: through,
            kind: ProviderObservationKind::KnownNoEffect {
                no_effect_evidence_id: format!("no-effect-{id}"),
            },
            observed_at_unix_ms: 20,
        }
    }

    fn success(record: &EffectRecord, id: &str, through: u32) -> ProviderObservation {
        ProviderObservation {
            observation_id: id.into(),
            execution_id: record.intent().execution_id.clone(),
            request_commitment: record.intent().request.request_commitment.clone(),
            covers_through_attempt_ordinal: through,
            kind: ProviderObservationKind::KnownSuccess {
                effect_commitment: "effect-1".into(),
                external_receipt_id: format!("receipt-{id}"),
                receipt_commitment: format!("receipt-commitment-{id}"),
            },
            observed_at_unix_ms: 40,
        }
    }

    #[test]
    fn facade_halts_late_success_that_contradicts_retry_basis_history() {
        let intent = EffectIntent::new(request(), 1).unwrap();
        let mut record = EffectRecord::new(intent).unwrap();
        record
            .begin_attempt("attempt-1", 10, RetryBasis::Initial)
            .unwrap();
        record
            .record_transport_failure("attempt-1", "timeout")
            .unwrap();

        let no_effect = no_effect(&record, "obs-no-effect-1", 1);
        assert_eq!(
            record.apply_observation(no_effect).unwrap(),
            ObservationDecision::Accepted
        );
        record
            .begin_attempt(
                "attempt-2",
                30,
                RetryBasis::ReconciledNoEffect {
                    observation_id: "obs-no-effect-1".into(),
                },
            )
            .unwrap();

        let late_success = success(&record, "obs-success-late", 1);
        assert!(matches!(
            record.apply_observation(late_success).unwrap(),
            ObservationDecision::IntegrityConflict {
                fault: EffectIntegrityFault::SuccessContradictsNoEffect { .. }
            }
        ));
        assert!(matches!(record.state(), EffectState::IntegrityHalted { .. }));
    }

    #[test]
    fn later_attempt_success_does_not_conflict_with_earlier_no_effect() {
        let intent = EffectIntent::new(request(), 1).unwrap();
        let mut record = EffectRecord::new(intent).unwrap();
        record
            .begin_attempt("attempt-1", 10, RetryBasis::Initial)
            .unwrap();
        record
            .record_transport_failure("attempt-1", "timeout")
            .unwrap();
        let no_effect = no_effect(&record, "obs-no-effect-1", 1);
        record.apply_observation(no_effect).unwrap();
        record
            .begin_attempt(
                "attempt-2",
                30,
                RetryBasis::ReconciledNoEffect {
                    observation_id: "obs-no-effect-1".into(),
                },
            )
            .unwrap();

        let success = success(&record, "obs-success-2", 2);
        assert_eq!(
            record.apply_observation(success).unwrap(),
            ObservationDecision::Accepted
        );
        assert!(matches!(record.state(), EffectState::KnownSuccess { .. }));
    }
}

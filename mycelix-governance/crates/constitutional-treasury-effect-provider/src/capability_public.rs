//! Capability-gated public API for MYC-CONST-003D1D-F0C.
//!
//! F0 established deterministic Treasury effect identity and guarded retry/
//! reconciliation transitions. F0C closes the remaining public trust gap:
//! callers can no longer assert provider replay qualification with a string.
//! Automatic replay from `UnknownOutcome` requires an opaque, effect-bound,
//! process-local capability with no public constructor.

mod f0 {
    include!("public.rs");
}

use std::collections::BTreeMap;

pub use f0::{
    EXECUTION_ID_PREFIX, INTENT_COMMITMENT_PREFIX, MAX_ID_LEN, MAX_REF_LEN,
    REQUEST_COMMITMENT_PREFIX, TREASURY_EFFECT_SCHEMA_VERSION, EffectIntegrityFault,
    EffectIntent, EffectState, ObservationDecision, ProviderObservation,
    ProviderObservationKind, RegistryDecision, RegistryIntegrityFault, TreasuryEffectError,
    TreasuryEffectRequest, TreasuryEffectResult, TreasuryEffectSubject,
};

const REPLAY_CAPABILITY_DOMAIN: &[u8] =
    b"MYCELIX-CONSTITUTIONAL-TREASURY-PROVIDER-REPLAY-CAPABILITY\0V1\0";
const REPLAY_CAPABILITY_EVIDENCE_PREFIX: &str = "provider-replay-capability-v1:";

fn require_nonempty(label: &str, value: &str) -> TreasuryEffectResult<()> {
    if value.trim().is_empty() {
        return Err(TreasuryEffectError::Violation(format!(
            "{label} must be non-empty"
        )));
    }
    Ok(())
}

fn push_str(hasher: &mut blake3::Hasher, value: &str) {
    let bytes = value.as_bytes();
    hasher.update(&(bytes.len() as u64).to_be_bytes());
    hasher.update(bytes);
}

/// Process-local proof that one exact provider profile has earned automatic
/// replay authority for one exact unresolved effect horizon.
///
/// Deliberately absent:
/// - `Serialize` / `Deserialize`;
/// - `Clone` / `Copy`;
/// - a public constructor;
/// - a constructor from a receipt ID or boolean.
///
/// A later provider-qualification tranche may add a verifier-owned minting path.
/// Until then, public callers cannot construct this type at all.
#[derive(Debug, PartialEq, Eq)]
pub struct QualifiedProviderReplayCapability {
    provider_profile_id: String,
    provider_profile_commitment: String,
    qualification_receipt_id: String,
    qualification_receipt_commitment: String,
    execution_id: String,
    request_commitment: String,
    covers_through_attempt_ordinal: u32,
    _sealed: (),
}

impl QualifiedProviderReplayCapability {
    pub fn provider_profile_id(&self) -> &str {
        &self.provider_profile_id
    }

    pub fn provider_profile_commitment(&self) -> &str {
        &self.provider_profile_commitment
    }

    pub fn qualification_receipt_id(&self) -> &str {
        &self.qualification_receipt_id
    }

    pub fn qualification_receipt_commitment(&self) -> &str {
        &self.qualification_receipt_commitment
    }

    pub fn execution_id(&self) -> &str {
        &self.execution_id
    }

    pub fn request_commitment(&self) -> &str {
        &self.request_commitment
    }

    pub fn covers_through_attempt_ordinal(&self) -> u32 {
        self.covers_through_attempt_ordinal
    }

    fn validate_for(&self, record: &f0::EffectRecord) -> TreasuryEffectResult<()> {
        require_nonempty("provider_profile_id", &self.provider_profile_id)?;
        require_nonempty(
            "provider_profile_commitment",
            &self.provider_profile_commitment,
        )?;
        require_nonempty("qualification_receipt_id", &self.qualification_receipt_id)?;
        require_nonempty(
            "qualification_receipt_commitment",
            &self.qualification_receipt_commitment,
        )?;

        if self.provider_profile_id != record.intent().request.subject.adapter_profile_id {
            return Err(TreasuryEffectError::Violation(
                "provider replay capability profile does not match effect adapter profile".into(),
            ));
        }
        if self.execution_id != record.intent().execution_id {
            return Err(TreasuryEffectError::Violation(
                "provider replay capability execution_id mismatch".into(),
            ));
        }
        if self.request_commitment != record.intent().request.request_commitment {
            return Err(TreasuryEffectError::Violation(
                "provider replay capability request commitment mismatch".into(),
            ));
        }

        match record.state() {
            EffectState::UnknownOutcome {
                through_attempt_ordinal,
                ..
            } if *through_attempt_ordinal == self.covers_through_attempt_ordinal => Ok(()),
            EffectState::UnknownOutcome { .. } => Err(TreasuryEffectError::Violation(
                "provider replay capability horizon does not match current UnknownOutcome".into(),
            )),
            _ => Err(TreasuryEffectError::Violation(
                "provider replay capability is valid only for the exact UnknownOutcome horizon"
                    .into(),
            )),
        }
    }

    fn evidence_id(&self) -> TreasuryEffectResult<String> {
        require_nonempty("provider_profile_id", &self.provider_profile_id)?;
        require_nonempty(
            "provider_profile_commitment",
            &self.provider_profile_commitment,
        )?;
        require_nonempty("qualification_receipt_id", &self.qualification_receipt_id)?;
        require_nonempty(
            "qualification_receipt_commitment",
            &self.qualification_receipt_commitment,
        )?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(REPLAY_CAPABILITY_DOMAIN);
        push_str(&mut hasher, &self.provider_profile_id);
        push_str(&mut hasher, &self.provider_profile_commitment);
        push_str(&mut hasher, &self.qualification_receipt_id);
        push_str(&mut hasher, &self.qualification_receipt_commitment);
        push_str(&mut hasher, &self.execution_id);
        push_str(&mut hasher, &self.request_commitment);
        hasher.update(&self.covers_through_attempt_ordinal.to_be_bytes());
        Ok(format!(
            "{REPLAY_CAPABILITY_EVIDENCE_PREFIX}{}",
            hasher.finalize().to_hex()
        ))
    }

    #[cfg(test)]
    fn mint_for_test(
        record: &f0::EffectRecord,
        provider_profile_id: &str,
        provider_profile_commitment: &str,
        qualification_receipt_id: &str,
        qualification_receipt_commitment: &str,
        covers_through_attempt_ordinal: u32,
    ) -> Self {
        Self {
            provider_profile_id: provider_profile_id.into(),
            provider_profile_commitment: provider_profile_commitment.into(),
            qualification_receipt_id: qualification_receipt_id.into(),
            qualification_receipt_commitment: qualification_receipt_commitment.into(),
            execution_id: record.intent().execution_id.clone(),
            request_commitment: record.intent().request.request_commitment.clone(),
            covers_through_attempt_ordinal,
            _sealed: (),
        }
    }
}

/// Public retry basis.
///
/// Unlike F0's private kernel enum, provider replay requires an opaque verifier-
/// owned capability rather than a caller-supplied string.
#[derive(Debug, PartialEq, Eq)]
pub enum RetryBasis {
    Initial,
    ProviderReplayQualified(QualifiedProviderReplayCapability),
    ReconciledNoEffect { observation_id: String },
}

impl RetryBasis {
    fn into_f0(self, record: &f0::EffectRecord) -> TreasuryEffectResult<f0::RetryBasis> {
        match self {
            Self::Initial => Ok(f0::RetryBasis::Initial),
            Self::ReconciledNoEffect { observation_id } => {
                Ok(f0::RetryBasis::ReconciledNoEffect { observation_id })
            }
            Self::ProviderReplayQualified(capability) => {
                capability.validate_for(record)?;
                Ok(f0::RetryBasis::ProviderReplayQualified {
                    capability_evidence_id: capability.evidence_id()?,
                })
            }
        }
    }
}

/// Audit-only retry evidence copied out of the private F0 attempt record.
///
/// This type is descriptive output. It cannot be supplied to authorize a retry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AttemptRetryEvidence {
    Initial,
    ProviderReplayQualified { capability_evidence_id: String },
    ReconciledNoEffect { observation_id: String },
}

/// Read-only public attempt snapshot. The private F0 retry enum never escapes the
/// capability facade through this audit surface.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EffectAttempt {
    pub attempt_id: String,
    pub attempt_ordinal: u32,
    pub execution_id: String,
    pub started_at_unix_ms: u64,
    pub retry_evidence: AttemptRetryEvidence,
}

impl From<f0::EffectAttempt> for EffectAttempt {
    fn from(value: f0::EffectAttempt) -> Self {
        let retry_evidence = match value.retry_basis {
            f0::RetryBasis::Initial => AttemptRetryEvidence::Initial,
            f0::RetryBasis::ProviderReplayQualified {
                capability_evidence_id,
            } => AttemptRetryEvidence::ProviderReplayQualified {
                capability_evidence_id,
            },
            f0::RetryBasis::ReconciledNoEffect { observation_id } => {
                AttemptRetryEvidence::ReconciledNoEffect { observation_id }
            }
        };
        Self {
            attempt_id: value.attempt_id,
            attempt_ordinal: value.attempt_ordinal,
            execution_id: value.execution_id,
            started_at_unix_ms: value.started_at_unix_ms,
            retry_evidence,
        }
    }
}

/// Capability-gated effect record.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EffectRecord {
    inner: f0::EffectRecord,
}

impl EffectRecord {
    pub fn new(intent: EffectIntent) -> TreasuryEffectResult<Self> {
        Ok(Self {
            inner: f0::EffectRecord::new(intent)?,
        })
    }

    fn from_inner(inner: f0::EffectRecord) -> Self {
        Self { inner }
    }

    pub fn intent(&self) -> &EffectIntent {
        self.inner.intent()
    }

    pub fn state(&self) -> &EffectState {
        self.inner.state()
    }

    pub fn attempts(&self) -> Vec<EffectAttempt> {
        self.inner
            .attempts()
            .iter()
            .cloned()
            .map(EffectAttempt::from)
            .collect()
    }

    pub fn observations(&self) -> &BTreeMap<String, ProviderObservation> {
        self.inner.observations()
    }

    pub fn begin_attempt(
        &mut self,
        attempt_id: impl Into<String>,
        started_at_unix_ms: u64,
        retry_basis: RetryBasis,
    ) -> TreasuryEffectResult<EffectAttempt> {
        let inner_basis = retry_basis.into_f0(&self.inner)?;
        self.inner
            .begin_attempt(attempt_id, started_at_unix_ms, inner_basis)
            .map(EffectAttempt::from)
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
        self.inner.apply_observation(observation)
    }

    pub fn validate_invariants(&self) -> TreasuryEffectResult<()> {
        self.inner.validate_invariants()
    }
}

/// Capability-gated registry.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct TreasuryEffectRegistry {
    inner: f0::TreasuryEffectRegistry,
}

impl Default for TreasuryEffectRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl TreasuryEffectRegistry {
    pub fn new() -> Self {
        Self {
            inner: f0::TreasuryEffectRegistry::new(),
        }
    }

    pub fn submit(
        &mut self,
        request: TreasuryEffectRequest,
        committed_at_unix_ms: u64,
    ) -> TreasuryEffectResult<RegistryDecision> {
        self.inner.submit(request, committed_at_unix_ms)
    }

    pub fn record(&self, execution_id: &str) -> Option<EffectRecord> {
        self.inner.record(execution_id).map(EffectRecord::from_inner)
    }

    pub fn by_action(&self, action_id: &str) -> Option<EffectRecord> {
        self.inner.by_action(action_id).map(EffectRecord::from_inner)
    }

    pub fn by_authorization(&self, authorization_id: &str) -> Option<EffectRecord> {
        self.inner
            .by_authorization(authorization_id)
            .map(EffectRecord::from_inner)
    }

    pub fn begin_attempt(
        &mut self,
        execution_id: &str,
        attempt_id: impl Into<String>,
        started_at_unix_ms: u64,
        retry_basis: RetryBasis,
    ) -> TreasuryEffectResult<EffectAttempt> {
        let snapshot = self.inner.record(execution_id).ok_or_else(|| {
            TreasuryEffectError::Violation("execution_id not found in effect registry".into())
        })?;
        let inner_basis = retry_basis.into_f0(&snapshot)?;
        self.inner
            .begin_attempt(execution_id, attempt_id, started_at_unix_ms, inner_basis)
            .map(EffectAttempt::from)
    }

    pub fn record_transport_failure(
        &mut self,
        execution_id: &str,
        attempt_id: &str,
        transport_error_ref: impl Into<String>,
    ) -> TreasuryEffectResult<()> {
        self.inner
            .record_transport_failure(execution_id, attempt_id, transport_error_ref)
    }

    pub fn apply_observation(
        &mut self,
        execution_id: &str,
        observation: ProviderObservation,
    ) -> TreasuryEffectResult<ObservationDecision> {
        self.inner.apply_observation(execution_id, observation)
    }

    pub fn validate_invariants(&self) -> TreasuryEffectResult<()> {
        self.inner.validate_invariants()
    }
}

#[cfg(test)]
mod capability_tests {
    use super::*;

    fn request(action: &str) -> TreasuryEffectRequest {
        TreasuryEffectRequest::new(TreasuryEffectSubject {
            operation_id: "op-1".into(),
            action_id: action.into(),
            proposal_id: "proposal-1".into(),
            claim_binding_commitment: "claim-binding-1".into(),
            action_commitment: format!("action-commitment-{action}"),
            authorization_id: format!("authorization-{action}"),
            authorization_subject_commitment: "authorization-subject-1".into(),
            treasury_descriptor_commitment: "treasury-descriptor-1".into(),
            allocation_subject_commitment: format!("allocation-{action}"),
            approval_projection_commitment: "approval-projection-1".into(),
            capacity_allocation_commitment: Some(format!("capacity-{action}")),
            recipient_did: "did:mycelix:recipient".into(),
            recipient_commitment: "recipient-1".into(),
            value_profile_id: "pending-sap-v1".into(),
            value_authority_commitment: "value-1".into(),
            policy_revision_commitment: "policy-1".into(),
            adapter_profile_id: "provider-profile-1".into(),
            effect_target_commitment: "target-1".into(),
        })
        .unwrap()
    }

    fn unknown_record(action: &str) -> EffectRecord {
        let intent = EffectIntent::new(request(action), 1).unwrap();
        let mut record = EffectRecord::new(intent).unwrap();
        record
            .begin_attempt("attempt-1", 10, RetryBasis::Initial)
            .unwrap();
        record
            .record_transport_failure("attempt-1", "timeout")
            .unwrap();
        record
    }

    fn capability(record: &EffectRecord, horizon: u32) -> QualifiedProviderReplayCapability {
        QualifiedProviderReplayCapability::mint_for_test(
            &record.inner,
            "provider-profile-1",
            "provider-profile-commitment-1",
            "qualification-receipt-1",
            "qualification-receipt-commitment-1",
            horizon,
        )
    }

    #[test]
    fn qualified_capability_can_replay_exact_unknown_horizon() {
        let mut record = unknown_record("action-1");
        let cap = capability(&record, 1);
        let first_execution_id = record.intent().execution_id.clone();
        let attempt = record
            .begin_attempt(
                "attempt-2",
                20,
                RetryBasis::ProviderReplayQualified(cap),
            )
            .unwrap();
        assert_eq!(attempt.attempt_ordinal, 2);
        assert_eq!(attempt.execution_id, first_execution_id);
        assert!(matches!(
            attempt.retry_evidence,
            AttemptRetryEvidence::ProviderReplayQualified { .. }
        ));
    }

    #[test]
    fn capability_horizon_must_match_exact_unknown_outcome() {
        let mut record = unknown_record("action-1");
        let cap = capability(&record, 2);
        let err = record
            .begin_attempt(
                "attempt-2",
                20,
                RetryBasis::ProviderReplayQualified(cap),
            )
            .unwrap_err();
        assert!(err.to_string().contains("horizon"));
    }

    #[test]
    fn capability_provider_profile_must_match_effect_adapter() {
        let mut record = unknown_record("action-1");
        let cap = QualifiedProviderReplayCapability::mint_for_test(
            &record.inner,
            "different-provider-profile",
            "provider-profile-commitment-1",
            "qualification-receipt-1",
            "qualification-receipt-commitment-1",
            1,
        );
        let err = record
            .begin_attempt(
                "attempt-2",
                20,
                RetryBasis::ProviderReplayQualified(cap),
            )
            .unwrap_err();
        assert!(err.to_string().contains("adapter profile"));
    }

    #[test]
    fn capability_is_bound_to_exact_execution_and_request() {
        let record_a = unknown_record("action-a");
        let mut record_b = unknown_record("action-b");
        let cap = capability(&record_a, 1);
        let err = record_b
            .begin_attempt(
                "attempt-2",
                20,
                RetryBasis::ProviderReplayQualified(cap),
            )
            .unwrap_err();
        assert!(err.to_string().contains("execution_id mismatch"));
    }

    #[test]
    fn no_effect_reconciliation_remains_available_without_provider_capability() {
        let intent = EffectIntent::new(request("action-1"), 1).unwrap();
        let mut record = EffectRecord::new(intent).unwrap();
        record
            .begin_attempt("attempt-1", 10, RetryBasis::Initial)
            .unwrap();
        record
            .record_transport_failure("attempt-1", "timeout")
            .unwrap();

        let observation = ProviderObservation {
            observation_id: "obs-no-effect-1".into(),
            execution_id: record.intent().execution_id.clone(),
            request_commitment: record.intent().request.request_commitment.clone(),
            covers_through_attempt_ordinal: 1,
            kind: ProviderObservationKind::KnownNoEffect {
                no_effect_evidence_id: "provider-no-effect-1".into(),
            },
            observed_at_unix_ms: 15,
        };
        record.apply_observation(observation).unwrap();
        let attempt = record
            .begin_attempt(
                "attempt-2",
                20,
                RetryBasis::ReconciledNoEffect {
                    observation_id: "obs-no-effect-1".into(),
                },
            )
            .unwrap();
        assert_eq!(attempt.attempt_ordinal, 2);
        assert!(matches!(
            attempt.retry_evidence,
            AttemptRetryEvidence::ReconciledNoEffect { .. }
        ));
    }
}

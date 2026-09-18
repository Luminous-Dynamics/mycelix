#![deny(unsafe_code)]

use constitutional_treasury_effect_provider::EffectIntent;
use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use thiserror::Error;

pub const PAYMENTS_PROVIDER_SCHEMA_VERSION: u16 = 1;
pub const PROVIDER_OPERATION_KEY_PREFIX: &str = "payments-provider-operation-v1:";
pub const PROVIDER_INTENT_COMMITMENT_PREFIX: &str = "payments-provider-intent-v1:";
const MAX_ID_LEN: usize = 256;
const MAX_REF_LEN: usize = 512;
const OPERATION_KEY_DOMAIN: &[u8] = b"MYCELIX-CONSTITUTIONAL-PAYMENTS-PROVIDER-OPERATION\0V1\0";
const INTENT_DOMAIN: &[u8] = b"MYCELIX-CONSTITUTIONAL-PAYMENTS-PROVIDER-INTENT\0V1\0";

#[derive(Debug, Error, Clone, PartialEq, Eq)]
pub enum PaymentsProviderError {
    #[error("{0}")]
    Violation(String),
    #[error("payments provider integrity halted: {0}")]
    IntegrityHalted(String),
}

pub type PaymentsProviderResult<T> = Result<T, PaymentsProviderError>;

fn violation(message: impl Into<String>) -> PaymentsProviderError {
    PaymentsProviderError::Violation(message.into())
}

fn require_opaque(label: &str, value: &str, max_len: usize) -> PaymentsProviderResult<()> {
    if value.trim().is_empty() || value.len() > max_len {
        return Err(violation(format!(
            "{label} must be non-empty and <= {max_len} bytes"
        )));
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

fn require_tagged_hash(label: &str, value: &str, prefix: &str) -> PaymentsProviderResult<()> {
    let digest = value
        .strip_prefix(prefix)
        .ok_or_else(|| violation(format!("{label} must use {prefix}<hex>")))?;
    if digest.len() != 64
        || !digest
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
    {
        return Err(violation(format!(
            "{label} must contain 64 lowercase hexadecimal digits"
        )));
    }
    Ok(())
}

/// Pure provider-operation intent derived from one exact F0 effect intent.
///
/// `committed_at_unix_ms` is diagnostic metadata and is excluded from semantic
/// operation and intent identity.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct PaymentProviderIntent {
    pub schema_version: u16,
    pub provider_profile_id: String,
    pub provider_profile_commitment: String,
    pub execution_id: String,
    pub request_commitment: String,
    pub provider_operation_key: String,
    pub intent_commitment: String,
    pub committed_at_unix_ms: u64,
}

impl PaymentProviderIntent {
    pub fn from_effect(
        effect: &EffectIntent,
        provider_profile_commitment: impl Into<String>,
        committed_at_unix_ms: u64,
    ) -> PaymentsProviderResult<Self> {
        effect
            .validate()
            .map_err(|e| violation(format!("invalid F0 effect intent: {e}")))?;
        let provider_profile_id = effect.request.subject.adapter_profile_id.clone();
        let provider_profile_commitment = provider_profile_commitment.into();
        require_opaque("provider_profile_id", &provider_profile_id, MAX_ID_LEN)?;
        require_opaque(
            "provider_profile_commitment",
            &provider_profile_commitment,
            MAX_REF_LEN,
        )?;

        let mut out = Self {
            schema_version: PAYMENTS_PROVIDER_SCHEMA_VERSION,
            provider_profile_id,
            provider_profile_commitment,
            execution_id: effect.execution_id.clone(),
            request_commitment: effect.request.request_commitment.clone(),
            provider_operation_key: String::new(),
            intent_commitment: String::new(),
            committed_at_unix_ms,
        };
        out.provider_operation_key = out.compute_operation_key()?;
        out.intent_commitment = out.compute_intent_commitment()?;
        Ok(out)
    }

    fn compute_operation_key(&self) -> PaymentsProviderResult<String> {
        require_opaque("provider_profile_id", &self.provider_profile_id, MAX_ID_LEN)?;
        require_opaque(
            "provider_profile_commitment",
            &self.provider_profile_commitment,
            MAX_REF_LEN,
        )?;
        require_opaque("execution_id", &self.execution_id, MAX_REF_LEN)?;
        require_opaque("request_commitment", &self.request_commitment, MAX_REF_LEN)?;
        let mut h = blake3::Hasher::new();
        h.update(OPERATION_KEY_DOMAIN);
        h.update(&self.schema_version.to_be_bytes());
        push_str(&mut h, &self.provider_profile_id);
        push_str(&mut h, &self.provider_profile_commitment);
        push_str(&mut h, &self.execution_id);
        push_str(&mut h, &self.request_commitment);
        Ok(tagged(PROVIDER_OPERATION_KEY_PREFIX, h.finalize()))
    }

    fn compute_intent_commitment(&self) -> PaymentsProviderResult<String> {
        let operation_key = self.compute_operation_key()?;
        let mut h = blake3::Hasher::new();
        h.update(INTENT_DOMAIN);
        h.update(&self.schema_version.to_be_bytes());
        push_str(&mut h, &operation_key);
        push_str(&mut h, &self.execution_id);
        push_str(&mut h, &self.request_commitment);
        push_str(&mut h, &self.provider_profile_id);
        push_str(&mut h, &self.provider_profile_commitment);
        Ok(tagged(PROVIDER_INTENT_COMMITMENT_PREFIX, h.finalize()))
    }

    pub fn validate(&self) -> PaymentsProviderResult<()> {
        if self.schema_version != PAYMENTS_PROVIDER_SCHEMA_VERSION {
            return Err(violation("payments provider schema version drift"));
        }
        require_tagged_hash(
            "provider_operation_key",
            &self.provider_operation_key,
            PROVIDER_OPERATION_KEY_PREFIX,
        )?;
        require_tagged_hash(
            "intent_commitment",
            &self.intent_commitment,
            PROVIDER_INTENT_COMMITMENT_PREFIX,
        )?;
        if self.provider_operation_key != self.compute_operation_key()? {
            return Err(violation(
                "provider_operation_key does not match exact F0 effect/profile binding",
            ));
        }
        if self.intent_commitment != self.compute_intent_commitment()? {
            return Err(violation(
                "intent_commitment does not match provider operation identity",
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub enum ProviderOutcomeEvidence {
    KnownSuccess {
        payment_id: String,
        provider_receipt_id: String,
        provider_receipt_commitment: String,
    },
    KnownNoEffect {
        no_effect_evidence_id: String,
    },
    UnknownOutcome {
        provider_evidence_id: String,
    },
}

impl ProviderOutcomeEvidence {
    fn validate(&self) -> PaymentsProviderResult<()> {
        match self {
            Self::KnownSuccess {
                payment_id,
                provider_receipt_id,
                provider_receipt_commitment,
            } => {
                require_opaque("payment_id", payment_id, MAX_ID_LEN)?;
                require_opaque("provider_receipt_id", provider_receipt_id, MAX_ID_LEN)?;
                require_opaque(
                    "provider_receipt_commitment",
                    provider_receipt_commitment,
                    MAX_REF_LEN,
                )
            }
            Self::KnownNoEffect {
                no_effect_evidence_id,
            } => require_opaque(
                "no_effect_evidence_id",
                no_effect_evidence_id,
                MAX_REF_LEN,
            ),
            Self::UnknownOutcome {
                provider_evidence_id,
            } => require_opaque("provider_evidence_id", provider_evidence_id, MAX_REF_LEN),
        }
    }
}

/// Provider-origin observation/query result for one exact provider operation.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProviderObservation {
    pub observation_id: String,
    pub provider_operation_key: String,
    pub execution_id: String,
    pub request_commitment: String,
    pub outcome: ProviderOutcomeEvidence,
    pub observed_at_unix_ms: u64,
}

impl ProviderObservation {
    pub fn validate_against(&self, intent: &PaymentProviderIntent) -> PaymentsProviderResult<()> {
        require_opaque("observation_id", &self.observation_id, MAX_ID_LEN)?;
        intent.validate()?;
        self.outcome.validate()?;
        if self.provider_operation_key != intent.provider_operation_key {
            return Err(violation("provider observation operation-key mismatch"));
        }
        if self.execution_id != intent.execution_id {
            return Err(violation("provider observation execution_id mismatch"));
        }
        if self.request_commitment != intent.request_commitment {
            return Err(violation("provider observation request commitment mismatch"));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProviderIntegrityFault {
    ObservationIdentityCollision { observation_id: String },
    SuccessContradictsNoEffect {
        success_observation_id: String,
        no_effect_observation_id: String,
    },
    NoEffectContradictsSuccess {
        no_effect_observation_id: String,
        success_observation_id: String,
    },
    ConflictingSuccessEvidence {
        prior_observation_id: String,
        conflicting_observation_id: String,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ProviderState {
    IntentCommitted,
    UnknownOutcome { observation_id: String },
    KnownSuccess {
        observation_id: String,
        payment_id: String,
        provider_receipt_id: String,
        provider_receipt_commitment: String,
    },
    KnownNoEffect { observation_id: String },
    IntegrityHalted { fault: ProviderIntegrityFault },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ObservationDecision {
    Accepted,
    ExistingSame,
    HistoricalOnly,
    IntegrityConflict { fault: ProviderIntegrityFault },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PaymentProviderOperation {
    intent: PaymentProviderIntent,
    state: ProviderState,
    observations: BTreeMap<String, ProviderObservation>,
}

impl PaymentProviderOperation {
    pub fn new(intent: PaymentProviderIntent) -> PaymentsProviderResult<Self> {
        intent.validate()?;
        Ok(Self {
            intent,
            state: ProviderState::IntentCommitted,
            observations: BTreeMap::new(),
        })
    }

    pub fn intent(&self) -> &PaymentProviderIntent {
        &self.intent
    }

    pub fn state(&self) -> &ProviderState {
        &self.state
    }

    pub fn observations(&self) -> &BTreeMap<String, ProviderObservation> {
        &self.observations
    }

    fn halt(&mut self, fault: ProviderIntegrityFault) -> ObservationDecision {
        self.state = ProviderState::IntegrityHalted {
            fault: fault.clone(),
        };
        ObservationDecision::IntegrityConflict { fault }
    }

    pub fn apply_observation(
        &mut self,
        observation: ProviderObservation,
    ) -> PaymentsProviderResult<ObservationDecision> {
        if let ProviderState::IntegrityHalted { fault } = &self.state {
            return Err(PaymentsProviderError::IntegrityHalted(format!("{fault:?}")));
        }
        observation.validate_against(&self.intent)?;

        if let Some(prior) = self.observations.get(&observation.observation_id) {
            if prior == &observation {
                return Ok(ObservationDecision::ExistingSame);
            }
            let fault = ProviderIntegrityFault::ObservationIdentityCollision {
                observation_id: observation.observation_id.clone(),
            };
            return Ok(self.halt(fault));
        }

        let prior_success = self.observations.values().find(|o| {
            matches!(o.outcome, ProviderOutcomeEvidence::KnownSuccess { .. })
        });
        let prior_no_effect = self.observations.values().find(|o| {
            matches!(o.outcome, ProviderOutcomeEvidence::KnownNoEffect { .. })
        });

        match &observation.outcome {
            ProviderOutcomeEvidence::KnownSuccess {
                payment_id,
                provider_receipt_id,
                provider_receipt_commitment,
            } => {
                if let Some(prior) = prior_no_effect {
                    let fault = ProviderIntegrityFault::SuccessContradictsNoEffect {
                        success_observation_id: observation.observation_id.clone(),
                        no_effect_observation_id: prior.observation_id.clone(),
                    };
                    return Ok(self.halt(fault));
                }
                if let Some(prior) = prior_success {
                    if prior.outcome != observation.outcome {
                        let fault = ProviderIntegrityFault::ConflictingSuccessEvidence {
                            prior_observation_id: prior.observation_id.clone(),
                            conflicting_observation_id: observation.observation_id.clone(),
                        };
                        return Ok(self.halt(fault));
                    }
                    self.observations
                        .insert(observation.observation_id.clone(), observation);
                    return Ok(ObservationDecision::HistoricalOnly);
                }
                self.state = ProviderState::KnownSuccess {
                    observation_id: observation.observation_id.clone(),
                    payment_id: payment_id.clone(),
                    provider_receipt_id: provider_receipt_id.clone(),
                    provider_receipt_commitment: provider_receipt_commitment.clone(),
                };
            }
            ProviderOutcomeEvidence::KnownNoEffect { .. } => {
                if let Some(prior) = prior_success {
                    let fault = ProviderIntegrityFault::NoEffectContradictsSuccess {
                        no_effect_observation_id: observation.observation_id.clone(),
                        success_observation_id: prior.observation_id.clone(),
                    };
                    return Ok(self.halt(fault));
                }
                self.state = ProviderState::KnownNoEffect {
                    observation_id: observation.observation_id.clone(),
                };
            }
            ProviderOutcomeEvidence::UnknownOutcome { .. } => {
                if matches!(
                    self.state,
                    ProviderState::KnownSuccess { .. } | ProviderState::KnownNoEffect { .. }
                ) {
                    self.observations
                        .insert(observation.observation_id.clone(), observation);
                    return Ok(ObservationDecision::HistoricalOnly);
                }
                self.state = ProviderState::UnknownOutcome {
                    observation_id: observation.observation_id.clone(),
                };
            }
        }

        self.observations
            .insert(observation.observation_id.clone(), observation);
        self.validate_invariants()?;
        Ok(ObservationDecision::Accepted)
    }

    pub fn validate_invariants(&self) -> PaymentsProviderResult<()> {
        self.intent.validate()?;
        for (id, observation) in &self.observations {
            if id != &observation.observation_id {
                return Err(violation("provider observation index key mismatch"));
            }
            observation.validate_against(&self.intent)?;
        }
        let successes: Vec<_> = self
            .observations
            .values()
            .filter(|o| matches!(o.outcome, ProviderOutcomeEvidence::KnownSuccess { .. }))
            .collect();
        let no_effects: Vec<_> = self
            .observations
            .values()
            .filter(|o| matches!(o.outcome, ProviderOutcomeEvidence::KnownNoEffect { .. }))
            .collect();
        if !successes.is_empty()
            && !no_effects.is_empty()
            && !matches!(self.state, ProviderState::IntegrityHalted { .. })
        {
            return Err(violation(
                "success and no-effect provider evidence cannot coexist without integrity halt",
            ));
        }
        Ok(())
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RegistryIntegrityFault {
    ExecutionIdentityReuse {
        execution_id: String,
        existing_operation_key: String,
        candidate_operation_key: String,
    },
    OperationKeyCollision { provider_operation_key: String },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum RegistryDecision {
    Created { provider_operation_key: String },
    ExistingSame { provider_operation_key: String },
    IntegrityConflict { fault: RegistryIntegrityFault },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct PaymentProviderRegistry {
    operations: BTreeMap<String, PaymentProviderOperation>,
    execution_index: BTreeMap<String, String>,
    integrity_fault: Option<RegistryIntegrityFault>,
}

impl Default for PaymentProviderRegistry {
    fn default() -> Self {
        Self::new()
    }
}

impl PaymentProviderRegistry {
    pub fn new() -> Self {
        Self {
            operations: BTreeMap::new(),
            execution_index: BTreeMap::new(),
            integrity_fault: None,
        }
    }

    pub fn submit(&mut self, intent: PaymentProviderIntent) -> PaymentsProviderResult<RegistryDecision> {
        if let Some(fault) = &self.integrity_fault {
            return Err(PaymentsProviderError::IntegrityHalted(format!("{fault:?}")));
        }
        intent.validate()?;
        let key = intent.provider_operation_key.clone();
        if let Some(existing) = self.operations.get(&key) {
            if existing.intent() == &intent {
                return Ok(RegistryDecision::ExistingSame {
                    provider_operation_key: key,
                });
            }
            let fault = RegistryIntegrityFault::OperationKeyCollision {
                provider_operation_key: key,
            };
            self.integrity_fault = Some(fault.clone());
            return Ok(RegistryDecision::IntegrityConflict { fault });
        }
        if let Some(existing_key) = self.execution_index.get(&intent.execution_id) {
            if existing_key != &key {
                let fault = RegistryIntegrityFault::ExecutionIdentityReuse {
                    execution_id: intent.execution_id.clone(),
                    existing_operation_key: existing_key.clone(),
                    candidate_operation_key: key,
                };
                self.integrity_fault = Some(fault.clone());
                return Ok(RegistryDecision::IntegrityConflict { fault });
            }
        }
        let operation = PaymentProviderOperation::new(intent.clone())?;
        self.execution_index
            .insert(intent.execution_id.clone(), key.clone());
        self.operations.insert(key.clone(), operation);
        self.validate_invariants()?;
        Ok(RegistryDecision::Created {
            provider_operation_key: key,
        })
    }

    pub fn operation(&self, key: &str) -> Option<&PaymentProviderOperation> {
        self.operations.get(key)
    }

    pub fn apply_observation(
        &mut self,
        key: &str,
        observation: ProviderObservation,
    ) -> PaymentsProviderResult<ObservationDecision> {
        if let Some(fault) = &self.integrity_fault {
            return Err(PaymentsProviderError::IntegrityHalted(format!("{fault:?}")));
        }
        let operation = self
            .operations
            .get_mut(key)
            .ok_or_else(|| violation("provider operation key not found"))?;
        operation.apply_observation(observation)
    }

    pub fn validate_invariants(&self) -> PaymentsProviderResult<()> {
        if self.operations.len() != self.execution_index.len() {
            return Err(violation(
                "each provider operation must have exactly one execution index entry",
            ));
        }
        for (key, op) in &self.operations {
            op.validate_invariants()?;
            if key != &op.intent().provider_operation_key {
                return Err(violation("provider operation registry key mismatch"));
            }
            if self.execution_index.get(&op.intent().execution_id) != Some(key) {
                return Err(violation("execution index does not bind provider operation"));
            }
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use constitutional_treasury_effect_provider::{EffectIntent, TreasuryEffectRequest, TreasuryEffectSubject};

    fn effect(action: &str, time: u64) -> EffectIntent {
        let request = TreasuryEffectRequest::new(TreasuryEffectSubject {
            operation_id: "op-1".into(),
            action_id: action.into(),
            proposal_id: "proposal-1".into(),
            claim_binding_commitment: "claim-binding-1".into(),
            action_commitment: format!("action-commitment-{action}"),
            authorization_id: format!("authorization-{action}"),
            authorization_subject_commitment: "authorization-subject-1".into(),
            treasury_descriptor_commitment: "treasury-1".into(),
            allocation_subject_commitment: format!("allocation-{action}"),
            approval_projection_commitment: "approval-1".into(),
            capacity_allocation_commitment: Some(format!("capacity-{action}")),
            recipient_did: "did:mycelix:recipient".into(),
            recipient_commitment: "recipient-1".into(),
            value_profile_id: "pending-sap-v1".into(),
            value_authority_commitment: "value-1".into(),
            policy_revision_commitment: "policy-1".into(),
            adapter_profile_id: "payments-provider-v1".into(),
            effect_target_commitment: "target-1".into(),
        })
        .unwrap();
        EffectIntent::new(request, time).unwrap()
    }

    fn intent(action: &str, time: u64) -> PaymentProviderIntent {
        PaymentProviderIntent::from_effect(&effect(action, time), "provider-profile-commitment-1", time)
            .unwrap()
    }

    fn observation(
        intent: &PaymentProviderIntent,
        id: &str,
        outcome: ProviderOutcomeEvidence,
    ) -> ProviderObservation {
        ProviderObservation {
            observation_id: id.into(),
            provider_operation_key: intent.provider_operation_key.clone(),
            execution_id: intent.execution_id.clone(),
            request_commitment: intent.request_commitment.clone(),
            outcome,
            observed_at_unix_ms: 99,
        }
    }

    #[test]
    fn wall_clock_is_not_provider_operation_identity() {
        let a = intent("action-1", 1);
        let b = intent("action-1", 999_999);
        assert_eq!(a.provider_operation_key, b.provider_operation_key);
        assert_eq!(a.intent_commitment, b.intent_commitment);
    }

    #[test]
    fn changed_effect_changes_provider_operation_key() {
        let a = intent("action-a", 1);
        let b = intent("action-b", 1);
        assert_ne!(a.execution_id, b.execution_id);
        assert_ne!(a.request_commitment, b.request_commitment);
        assert_ne!(a.provider_operation_key, b.provider_operation_key);
    }

    #[test]
    fn changed_provider_profile_commitment_changes_key() {
        let e = effect("action-1", 1);
        let a = PaymentProviderIntent::from_effect(&e, "profile-a", 1).unwrap();
        let b = PaymentProviderIntent::from_effect(&e, "profile-b", 1).unwrap();
        assert_ne!(a.provider_operation_key, b.provider_operation_key);
    }

    #[test]
    fn same_exact_intent_is_idempotent_in_registry() {
        let i = intent("action-1", 1);
        let mut r = PaymentProviderRegistry::new();
        assert!(matches!(r.submit(i.clone()).unwrap(), RegistryDecision::Created { .. }));
        assert!(matches!(r.submit(i).unwrap(), RegistryDecision::ExistingSame { .. }));
    }

    #[test]
    fn same_execution_cannot_change_provider_profile_commitment() {
        let e = effect("action-1", 1);
        let a = PaymentProviderIntent::from_effect(&e, "profile-a", 1).unwrap();
        let b = PaymentProviderIntent::from_effect(&e, "profile-b", 1).unwrap();
        let mut r = PaymentProviderRegistry::new();
        r.submit(a).unwrap();
        assert!(matches!(
            r.submit(b).unwrap(),
            RegistryDecision::IntegrityConflict {
                fault: RegistryIntegrityFault::ExecutionIdentityReuse { .. }
            }
        ));
    }

    #[test]
    fn unknown_can_resolve_to_success() {
        let i = intent("action-1", 1);
        let mut op = PaymentProviderOperation::new(i.clone()).unwrap();
        let unknown = observation(
            &i,
            "obs-unknown",
            ProviderOutcomeEvidence::UnknownOutcome {
                provider_evidence_id: "query-unknown-1".into(),
            },
        );
        assert_eq!(op.apply_observation(unknown).unwrap(), ObservationDecision::Accepted);
        assert!(matches!(op.state(), ProviderState::UnknownOutcome { .. }));
        let success = observation(
            &i,
            "obs-success",
            ProviderOutcomeEvidence::KnownSuccess {
                payment_id: "payment-1".into(),
                provider_receipt_id: "receipt-1".into(),
                provider_receipt_commitment: "receipt-commitment-1".into(),
            },
        );
        assert_eq!(op.apply_observation(success).unwrap(), ObservationDecision::Accepted);
        assert!(matches!(op.state(), ProviderState::KnownSuccess { .. }));
    }

    #[test]
    fn unknown_can_resolve_to_known_no_effect() {
        let i = intent("action-1", 1);
        let mut op = PaymentProviderOperation::new(i.clone()).unwrap();
        op.apply_observation(observation(
            &i,
            "obs-unknown",
            ProviderOutcomeEvidence::UnknownOutcome {
                provider_evidence_id: "query-unknown-1".into(),
            },
        ))
        .unwrap();
        op.apply_observation(observation(
            &i,
            "obs-none",
            ProviderOutcomeEvidence::KnownNoEffect {
                no_effect_evidence_id: "query-none-1".into(),
            },
        ))
        .unwrap();
        assert!(matches!(op.state(), ProviderState::KnownNoEffect { .. }));
    }

    #[test]
    fn success_and_no_effect_conflict_halts() {
        let i = intent("action-1", 1);
        let mut op = PaymentProviderOperation::new(i.clone()).unwrap();
        op.apply_observation(observation(
            &i,
            "obs-success",
            ProviderOutcomeEvidence::KnownSuccess {
                payment_id: "payment-1".into(),
                provider_receipt_id: "receipt-1".into(),
                provider_receipt_commitment: "receipt-commitment-1".into(),
            },
        ))
        .unwrap();
        let result = op
            .apply_observation(observation(
                &i,
                "obs-none",
                ProviderOutcomeEvidence::KnownNoEffect {
                    no_effect_evidence_id: "none-1".into(),
                },
            ))
            .unwrap();
        assert!(matches!(result, ObservationDecision::IntegrityConflict { .. }));
        assert!(matches!(op.state(), ProviderState::IntegrityHalted { .. }));
    }

    #[test]
    fn conflicting_success_receipts_halt() {
        let i = intent("action-1", 1);
        let mut op = PaymentProviderOperation::new(i.clone()).unwrap();
        op.apply_observation(observation(
            &i,
            "obs-success-1",
            ProviderOutcomeEvidence::KnownSuccess {
                payment_id: "payment-1".into(),
                provider_receipt_id: "receipt-1".into(),
                provider_receipt_commitment: "receipt-commitment-1".into(),
            },
        ))
        .unwrap();
        let result = op
            .apply_observation(observation(
                &i,
                "obs-success-2",
                ProviderOutcomeEvidence::KnownSuccess {
                    payment_id: "payment-2".into(),
                    provider_receipt_id: "receipt-2".into(),
                    provider_receipt_commitment: "receipt-commitment-2".into(),
                },
            ))
            .unwrap();
        assert!(matches!(result, ObservationDecision::IntegrityConflict { .. }));
    }

    #[test]
    fn observation_must_bind_exact_request_and_execution() {
        let i = intent("action-1", 1);
        let mut op = PaymentProviderOperation::new(i.clone()).unwrap();
        let mut o = observation(
            &i,
            "obs-1",
            ProviderOutcomeEvidence::UnknownOutcome {
                provider_evidence_id: "query-1".into(),
            },
        );
        o.request_commitment.push_str("-wrong");
        assert!(op.apply_observation(o).is_err());
    }
}

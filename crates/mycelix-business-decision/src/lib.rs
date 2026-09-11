// Copyright (C) 2024-2026 Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Evidence-bearing decision and execution lineage for the Mycelix Business Fabric.
//!
//! This crate deliberately separates recommendation, authorization, execution attempts,
//! provider acknowledgements, economic reconciliation, and observed outcomes. It mints no
//! authority and owns no external business-domain truth.

use std::collections::BTreeSet;

use mycelix_business_coordination::{CoordinationEnvelope, UnknownOutcomePolicy};
use mycelix_business_core::{
    ActionContractRef, AuthorizedIntentRef, CapabilityRef, DecisionFrontiers, Digest32,
    EstimateRef, ExecutionAttemptRef, ExecutionReceiptRef, ForecastRef, ObservationRef,
    PreparedAction, ProposalRef, ReconciliationRef, ReferenceId, SubjectRef,
};

/// Exact reference to one immutable decision capsule.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecisionCapsuleRef {
    pub id: ReferenceId,
    pub digest: Digest32,
}

/// Human-facing explanation is deliberately separate from machine authorization fields.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HumanExplanationRef(pub ReferenceId);

/// Immutable reasoning lineage for one recommendation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecisionCapsule {
    pub capsule_ref: DecisionCapsuleRef,
    pub subject: SubjectRef,
    pub action_contract: ActionContractRef,
    pub frontiers: DecisionFrontiers,
    pub observations: BTreeSet<ObservationRef>,
    pub estimates: BTreeSet<EstimateRef>,
    pub forecasts: BTreeSet<ForecastRef>,
    pub assumptions: BTreeSet<ReferenceId>,
    pub conflicts: BTreeSet<ReferenceId>,
    pub alternatives: BTreeSet<ProposalRef>,
    pub recommendation: ProposalRef,
    /// Audit declaration only. The Action Contract remains authoritative for required capabilities.
    pub required_capabilities: BTreeSet<CapabilityRef>,
    pub model_lineage: ReferenceId,
    pub objective_contract_digest: Digest32,
    pub human_explanation: Option<HumanExplanationRef>,
    pub created_at_unix_ms: u64,
    pub valid_until_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DecisionCapsuleError {
    InvalidWindow,
    NoEpistemicInputs,
    RecommendationNotAlternative,
    ContractMismatch,
    PreparedDecisionMismatch,
    CapabilityContractMismatch,
    PreparedActionOutlivesDecision,
}

impl DecisionCapsule {
    pub fn validate(&self) -> Result<(), DecisionCapsuleError> {
        if self.created_at_unix_ms >= self.valid_until_unix_ms {
            return Err(DecisionCapsuleError::InvalidWindow);
        }
        if self.observations.is_empty() && self.estimates.is_empty() && self.forecasts.is_empty() {
            return Err(DecisionCapsuleError::NoEpistemicInputs);
        }
        if !self.alternatives.contains(&self.recommendation) {
            return Err(DecisionCapsuleError::RecommendationNotAlternative);
        }
        Ok(())
    }

    /// Ensure the reasoning capsule cannot silently redefine execution requirements.
    pub fn validate_against_coordination(
        &self,
        coordination: &CoordinationEnvelope,
    ) -> Result<(), DecisionCapsuleError> {
        self.validate()?;
        if self.action_contract != coordination.contract.contract_ref {
            return Err(DecisionCapsuleError::ContractMismatch);
        }
        if self.capsule_ref.id != coordination.prepared.decision_capsule {
            return Err(DecisionCapsuleError::PreparedDecisionMismatch);
        }
        if self.required_capabilities != coordination.contract.required_capabilities {
            return Err(DecisionCapsuleError::CapabilityContractMismatch);
        }
        if coordination.prepared.expires_at_unix_ms > self.valid_until_unix_ms {
            return Err(DecisionCapsuleError::PreparedActionOutlivesDecision);
        }
        Ok(())
    }
}

/// Evidence that an external authority approved one exact intent derived from a decision.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorizationBinding {
    pub decision: DecisionCapsuleRef,
    pub authorized_intent: AuthorizedIntentRef,
    pub action_contract: ActionContractRef,
    pub intent_digest: Digest32,
    pub authority_lease_id: ReferenceId,
    pub authority_epoch: u64,
    pub fencing_token: u64,
    pub approved_at_unix_ms: u64,
    pub approval_evidence: BTreeSet<ReferenceId>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum AuthorizationBindingError {
    DecisionMismatch,
    ContractMismatch,
    IntentMismatch,
    LeaseMismatch,
    AuthorityEpochMismatch,
    FencingTokenMismatch,
    ApprovalBeforeLease,
    ApprovalAfterLease,
    ApprovalAfterPreparation,
    MissingApprovalEvidence,
}

impl AuthorizationBinding {
    /// Validate that this authorization binds the exact prepared action it claims to authorize.
    pub fn validate_prepared(
        &self,
        prepared: &PreparedAction,
    ) -> Result<(), AuthorizationBindingError> {
        if prepared.decision_capsule != self.decision.id {
            return Err(AuthorizationBindingError::DecisionMismatch);
        }
        if prepared.action_contract != self.action_contract {
            return Err(AuthorizationBindingError::ContractMismatch);
        }
        if prepared.intent_digest != self.intent_digest {
            return Err(AuthorizationBindingError::IntentMismatch);
        }
        if prepared.authority_lease.lease_id != self.authority_lease_id {
            return Err(AuthorizationBindingError::LeaseMismatch);
        }
        if prepared.authority_lease.authority_epoch != self.authority_epoch {
            return Err(AuthorizationBindingError::AuthorityEpochMismatch);
        }
        if prepared.authority_lease.fencing_token != self.fencing_token {
            return Err(AuthorizationBindingError::FencingTokenMismatch);
        }
        if self.approved_at_unix_ms < prepared.authority_lease.issued_at_unix_ms {
            return Err(AuthorizationBindingError::ApprovalBeforeLease);
        }
        if self.approved_at_unix_ms >= prepared.authority_lease.expires_at_unix_ms {
            return Err(AuthorizationBindingError::ApprovalAfterLease);
        }
        if self.approved_at_unix_ms > prepared.prepared_at_unix_ms {
            return Err(AuthorizationBindingError::ApprovalAfterPreparation);
        }
        if self.approval_evidence.is_empty() {
            return Err(AuthorizationBindingError::MissingApprovalEvidence);
        }
        Ok(())
    }
}

/// One concrete attempt to execute an already authorized and coordinated intent.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExecutionAttemptRecord {
    pub attempt: ExecutionAttemptRef,
    pub decision: DecisionCapsuleRef,
    pub authorized_intent: AuthorizedIntentRef,
    pub coordination_digest: Digest32,
    pub idempotency_key: ReferenceId,
    pub provider: ReferenceId,
    pub started_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExecutionAttemptBindingError {
    DecisionMismatch,
    AuthorizedIntentMismatch,
    CoordinationMismatch,
    IdempotencyMismatch,
    AttemptBeforePreparation,
    AttemptAfterExpiry,
}

impl ExecutionAttemptRecord {
    pub fn validate_binding(
        &self,
        authorization: &AuthorizationBinding,
        coordination: &CoordinationEnvelope,
    ) -> Result<(), ExecutionAttemptBindingError> {
        if self.decision != authorization.decision {
            return Err(ExecutionAttemptBindingError::DecisionMismatch);
        }
        if self.authorized_intent != authorization.authorized_intent {
            return Err(ExecutionAttemptBindingError::AuthorizedIntentMismatch);
        }
        if self.coordination_digest != coordination.coordination_digest {
            return Err(ExecutionAttemptBindingError::CoordinationMismatch);
        }
        if self.idempotency_key != coordination.prepared.idempotency_key {
            return Err(ExecutionAttemptBindingError::IdempotencyMismatch);
        }
        if self.started_at_unix_ms < coordination.prepared.prepared_at_unix_ms {
            return Err(ExecutionAttemptBindingError::AttemptBeforePreparation);
        }
        if self.started_at_unix_ms >= coordination.prepared.expires_at_unix_ms {
            return Err(ExecutionAttemptBindingError::AttemptAfterExpiry);
        }
        Ok(())
    }
}

/// Transport-level result only. This is not an economic-effect claim.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransportDisposition {
    Delivered,
    Failed,
    Unknown,
}

/// Provider acknowledgement only. `Accepted` does not imply economic finality.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProviderDisposition {
    Accepted,
    Rejected,
    Pending,
    Unknown,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ExecutionReceiptRecord {
    pub receipt: ExecutionReceiptRef,
    pub attempt: ExecutionAttemptRef,
    pub transport: TransportDisposition,
    pub provider: ProviderDisposition,
    pub provider_operation_id: Option<ReferenceId>,
    pub observed_at_unix_ms: u64,
    pub evidence: BTreeSet<ReferenceId>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ExecutionReceiptError {
    AttemptMismatch,
    ReceiptBeforeAttempt,
    MissingEvidence,
}

impl ExecutionReceiptRecord {
    pub fn validate(&self, attempt: &ExecutionAttemptRecord) -> Result<(), ExecutionReceiptError> {
        if self.attempt != attempt.attempt {
            return Err(ExecutionReceiptError::AttemptMismatch);
        }
        if self.observed_at_unix_ms < attempt.started_at_unix_ms {
            return Err(ExecutionReceiptError::ReceiptBeforeAttempt);
        }
        if self.evidence.is_empty() {
            return Err(ExecutionReceiptError::MissingEvidence);
        }
        Ok(())
    }
}

/// Reconciled economic effect. Provider acknowledgement alone must never construct this value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EconomicEffectDisposition {
    Applied,
    NotApplied,
    StillUnknown,
    Compensated,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct ReconciliationRecord {
    pub reconciliation: ReconciliationRef,
    pub attempt: ExecutionAttemptRef,
    pub receipt: Option<ExecutionReceiptRef>,
    pub disposition: EconomicEffectDisposition,
    pub evidence: BTreeSet<ReferenceId>,
    pub compensation_attempt: Option<ExecutionAttemptRef>,
    pub reconciled_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ReconciliationError {
    AttemptMismatch,
    ReceiptMismatch,
    MissingReceiptRecord,
    ReconciliationBeforeAttempt,
    MissingEvidence,
    CompensationAttemptRequired,
    UnexpectedCompensationAttempt,
    SelfCompensation,
}

impl ReconciliationRecord {
    pub fn validate(
        &self,
        attempt: &ExecutionAttemptRecord,
        receipt: Option<&ExecutionReceiptRecord>,
    ) -> Result<(), ReconciliationError> {
        if self.attempt != attempt.attempt {
            return Err(ReconciliationError::AttemptMismatch);
        }
        if self.reconciled_at_unix_ms < attempt.started_at_unix_ms {
            return Err(ReconciliationError::ReconciliationBeforeAttempt);
        }
        if self.evidence.is_empty() {
            return Err(ReconciliationError::MissingEvidence);
        }

        match (&self.receipt, receipt) {
            (Some(expected), Some(actual)) => {
                if &actual.receipt != expected || actual.attempt != self.attempt {
                    return Err(ReconciliationError::ReceiptMismatch);
                }
            }
            (Some(_), None) => return Err(ReconciliationError::MissingReceiptRecord),
            (None, Some(_)) | (None, None) => {}
        }

        match (self.disposition, &self.compensation_attempt) {
            (EconomicEffectDisposition::Compensated, Some(compensation)) => {
                if compensation == &self.attempt {
                    return Err(ReconciliationError::SelfCompensation);
                }
            }
            (EconomicEffectDisposition::Compensated, None) => {
                return Err(ReconciliationError::CompensationAttemptRequired);
            }
            (_, Some(_)) => return Err(ReconciliationError::UnexpectedCompensationAttempt),
            (_, None) => {}
        }

        Ok(())
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RetryDisposition {
    /// Retry only with the exact same provider-supported idempotency identity.
    RetryWithSameIdempotencyKey,
    /// Reconciliation established that no effect occurred, so a fresh attempt is allowed.
    RetryAfterConfirmedNotApplied,
    /// Do not risk a duplicate effect; reconcile the external state first.
    ReconcileFirst,
    /// An economic effect already occurred or was compensated; do not replay the original action.
    DoNotRetry,
}

/// Determine the safe retry posture without converting uncertainty into failure.
pub fn retry_disposition(
    policy: UnknownOutcomePolicy,
    provider_supports_stable_idempotency: bool,
    reconciliation: Option<&ReconciliationRecord>,
) -> RetryDisposition {
    match reconciliation.map(|value| value.disposition) {
        Some(EconomicEffectDisposition::Applied | EconomicEffectDisposition::Compensated) => {
            RetryDisposition::DoNotRetry
        }
        Some(EconomicEffectDisposition::NotApplied) => {
            RetryDisposition::RetryAfterConfirmedNotApplied
        }
        Some(EconomicEffectDisposition::StillUnknown) | None => match policy {
            UnknownOutcomePolicy::IdempotentRetryOnly if provider_supports_stable_idempotency => {
                RetryDisposition::RetryWithSameIdempotencyKey
            }
            UnknownOutcomePolicy::IdempotentRetryOnly | UnknownOutcomePolicy::ReconcileBeforeRetry => {
                RetryDisposition::ReconcileFirst
            }
        },
    }
}

/// Outcome and calibration lineage after reconciliation.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DecisionOutcomeRecord {
    pub decision: DecisionCapsuleRef,
    pub reconciliation: ReconciliationRef,
    pub outcome_observations: BTreeSet<ObservationRef>,
    pub observed_at_unix_ms: u64,
    pub evaluation_horizon_ms: u64,
    pub calibration_evidence: Option<ReferenceId>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum DecisionOutcomeError {
    DecisionMismatch,
    ReconciliationMismatch,
    AttemptMismatch,
    EconomicEffectStillUnknown,
    NoOutcomeObservations,
    ZeroEvaluationHorizon,
    OutcomeBeforeReconciliation,
}

impl DecisionOutcomeRecord {
    pub fn validate(
        &self,
        reconciliation: &ReconciliationRecord,
        attempt: &ExecutionAttemptRecord,
    ) -> Result<(), DecisionOutcomeError> {
        if self.decision != attempt.decision {
            return Err(DecisionOutcomeError::DecisionMismatch);
        }
        if self.reconciliation != reconciliation.reconciliation {
            return Err(DecisionOutcomeError::ReconciliationMismatch);
        }
        if reconciliation.attempt != attempt.attempt {
            return Err(DecisionOutcomeError::AttemptMismatch);
        }
        if reconciliation.disposition == EconomicEffectDisposition::StillUnknown {
            return Err(DecisionOutcomeError::EconomicEffectStillUnknown);
        }
        if self.outcome_observations.is_empty() {
            return Err(DecisionOutcomeError::NoOutcomeObservations);
        }
        if self.evaluation_horizon_ms == 0 {
            return Err(DecisionOutcomeError::ZeroEvaluationHorizon);
        }
        if self.observed_at_unix_ms < reconciliation.reconciled_at_unix_ms {
            return Err(DecisionOutcomeError::OutcomeBeforeReconciliation);
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_business_coordination::{
        ActionContract, AggregateCheckRef, AggregatePolicyKey, FreshnessRequirement,
        ObservationAge, ReversibilityClass,
    };
    use mycelix_business_core::{
        AuthorityLeaseRef, AuthorityScope, BudgetLimit, ReservationId, ReservationRef, RiskClass,
        ScopeRef,
    };

    fn id(value: &str) -> ReferenceId {
        ReferenceId::new(value).unwrap()
    }

    fn proposal(value: &str) -> ProposalRef {
        ProposalRef(id(value))
    }

    fn observation(value: &str) -> ObservationRef {
        ObservationRef(id(value))
    }

    fn capability(value: &str) -> CapabilityRef {
        CapabilityRef(id(value))
    }

    fn subject(value: &str) -> SubjectRef {
        SubjectRef(id(value))
    }

    fn decision_ref() -> DecisionCapsuleRef {
        DecisionCapsuleRef {
            id: id("decision:1"),
            digest: Digest32::repeat(21),
        }
    }

    fn contract_ref() -> ActionContractRef {
        ActionContractRef {
            semantic_id: id("mycelix.procurement.place-order.v1"),
            digest: Digest32::repeat(1),
        }
    }

    fn capsule() -> DecisionCapsule {
        let recommended = proposal("proposal:order-20kg");
        DecisionCapsule {
            capsule_ref: decision_ref(),
            subject: subject("restaurant:a"),
            action_contract: contract_ref(),
            frontiers: DecisionFrontiers::default(),
            observations: BTreeSet::from([observation("observation:inventory:1")]),
            estimates: BTreeSet::new(),
            forecasts: BTreeSet::new(),
            assumptions: BTreeSet::from([id("assumption:delivery-window")]),
            conflicts: BTreeSet::new(),
            alternatives: BTreeSet::from([proposal("proposal:no-order"), recommended.clone()]),
            recommendation: recommended,
            required_capabilities: BTreeSet::from([capability("procurement:place-order")]),
            model_lineage: id("model:symthaea-demand:v17"),
            objective_contract_digest: Digest32::repeat(22),
            human_explanation: Some(HumanExplanationRef(id("explanation:1"))),
            created_at_unix_ms: 1_000,
            valid_until_unix_ms: 1_600,
        }
    }

    fn prepared() -> PreparedAction {
        PreparedAction {
            action_contract: contract_ref(),
            decision_capsule: id("decision:1"),
            intent_digest: Digest32::repeat(2),
            frontiers: DecisionFrontiers::default(),
            authority_lease: AuthorityLeaseRef {
                lease_id: id("authority:1"),
                authority_epoch: 7,
                sequence: 1,
                fencing_token: 11,
                scope: ScopeRef(id("scope:restaurant:a")),
                issued_at_unix_ms: 900,
                expires_at_unix_ms: 2_000,
            },
            reservations: vec![ReservationRef {
                domain: id("finance"),
                reservation_id: ReservationId(id("reservation:finance:1")),
                subject: subject("budget:restaurant:a"),
                digest: Digest32::repeat(3),
                expires_at_unix_ms: 1_900,
            }],
            idempotency_key: id("order:1"),
            prepared_at_unix_ms: 1_200,
            expires_at_unix_ms: 1_500,
        }
    }

    fn coordination() -> CoordinationEnvelope {
        CoordinationEnvelope {
            contract: ActionContract {
                contract_ref: contract_ref(),
                required_reservation_domains: BTreeSet::from([id("finance")]),
                required_capabilities: BTreeSet::from([capability("procurement:place-order")]),
                freshness: vec![FreshnessRequirement {
                    domain: id("inventory"),
                    maximum_age_ms: 30_000,
                }],
                aggregate_policy_keys: vec![AggregatePolicyKey {
                    key: id("aggregate:procurement:24h"),
                    window_ms: 86_400_000,
                }],
                reversibility: ReversibilityClass::OperationallyReversible,
                maximum_prepared_lifetime_ms: 600,
                unknown_outcome_policy: UnknownOutcomePolicy::ReconcileBeforeRetry,
            },
            prepared: prepared(),
            authority_scope: AuthorityScope {
                capabilities: BTreeSet::from([capability("procurement:place-order")]),
                subjects: BTreeSet::from([subject("restaurant:a")]),
                risk_ceiling: RiskClass::Moderate,
                budget: BudgetLimit::Limited(2_000),
                expires_at_unix_ms: 2_000,
            },
            observation_ages: vec![ObservationAge {
                domain: id("inventory"),
                age_ms: 10_000,
            }],
            aggregate_checks: vec![AggregateCheckRef {
                key: id("aggregate:procurement:24h"),
                digest: Digest32::repeat(4),
            }],
            coordination_digest: Digest32::repeat(5),
        }
    }

    fn authorization() -> AuthorizationBinding {
        AuthorizationBinding {
            decision: decision_ref(),
            authorized_intent: AuthorizedIntentRef(id("intent:1")),
            action_contract: contract_ref(),
            intent_digest: Digest32::repeat(2),
            authority_lease_id: id("authority:1"),
            authority_epoch: 7,
            fencing_token: 11,
            approved_at_unix_ms: 1_100,
            approval_evidence: BTreeSet::from([id("approval:signature:1")]),
        }
    }

    fn attempt() -> ExecutionAttemptRecord {
        ExecutionAttemptRecord {
            attempt: ExecutionAttemptRef(id("attempt:1")),
            decision: decision_ref(),
            authorized_intent: AuthorizedIntentRef(id("intent:1")),
            coordination_digest: Digest32::repeat(5),
            idempotency_key: id("order:1"),
            provider: id("supplier:api:a"),
            started_at_unix_ms: 1_300,
        }
    }

    fn receipt() -> ExecutionReceiptRecord {
        ExecutionReceiptRecord {
            receipt: ExecutionReceiptRef(id("receipt:1")),
            attempt: ExecutionAttemptRef(id("attempt:1")),
            transport: TransportDisposition::Delivered,
            provider: ProviderDisposition::Accepted,
            provider_operation_id: Some(id("supplier-order:abc")),
            observed_at_unix_ms: 1_320,
            evidence: BTreeSet::from([id("provider-response-digest:1")]),
        }
    }

    fn reconciliation(disposition: EconomicEffectDisposition) -> ReconciliationRecord {
        ReconciliationRecord {
            reconciliation: ReconciliationRef(id("reconciliation:1")),
            attempt: ExecutionAttemptRef(id("attempt:1")),
            receipt: Some(ExecutionReceiptRef(id("receipt:1"))),
            disposition,
            evidence: BTreeSet::from([id("supplier-query:1")]),
            compensation_attempt: None,
            reconciled_at_unix_ms: 1_400,
        }
    }

    #[test]
    fn decision_capsule_cannot_redefine_action_contract_capabilities() {
        let coordination = coordination();
        let mut value = capsule();
        assert_eq!(value.validate_against_coordination(&coordination), Ok(()));
        value
            .required_capabilities
            .insert(capability("treasury:borrow"));
        assert_eq!(
            value.validate_against_coordination(&coordination),
            Err(DecisionCapsuleError::CapabilityContractMismatch)
        );
    }

    #[test]
    fn authorization_binds_exact_prepared_intent_and_authority_epoch() {
        let prepared = prepared();
        let mut value = authorization();
        assert_eq!(value.validate_prepared(&prepared), Ok(()));
        value.authority_epoch = 8;
        assert_eq!(
            value.validate_prepared(&prepared),
            Err(AuthorizationBindingError::AuthorityEpochMismatch)
        );
    }

    #[test]
    fn execution_attempt_binds_coordination_and_idempotency_identity() {
        let coordination = coordination();
        let authorization = authorization();
        let mut value = attempt();
        assert_eq!(value.validate_binding(&authorization, &coordination), Ok(()));
        value.idempotency_key = id("order:different");
        assert_eq!(
            value.validate_binding(&authorization, &coordination),
            Err(ExecutionAttemptBindingError::IdempotencyMismatch)
        );
    }

    #[test]
    fn provider_acceptance_does_not_establish_economic_effect() {
        let attempt = attempt();
        let receipt = receipt();
        assert_eq!(receipt.validate(&attempt), Ok(()));

        let unknown = reconciliation(EconomicEffectDisposition::StillUnknown);
        assert_eq!(unknown.validate(&attempt, Some(&receipt)), Ok(()));
        assert_eq!(
            retry_disposition(
                UnknownOutcomePolicy::ReconcileBeforeRetry,
                true,
                Some(&unknown)
            ),
            RetryDisposition::ReconcileFirst
        );
    }

    #[test]
    fn idempotent_retry_requires_provider_support_and_exact_policy() {
        assert_eq!(
            retry_disposition(UnknownOutcomePolicy::IdempotentRetryOnly, true, None),
            RetryDisposition::RetryWithSameIdempotencyKey
        );
        assert_eq!(
            retry_disposition(UnknownOutcomePolicy::IdempotentRetryOnly, false, None),
            RetryDisposition::ReconcileFirst
        );
    }

    #[test]
    fn compensation_preserves_original_attempt_and_requires_distinct_attempt() {
        let attempt = attempt();
        let receipt = receipt();
        let mut value = reconciliation(EconomicEffectDisposition::Compensated);
        value.compensation_attempt = Some(ExecutionAttemptRef(id("attempt:compensation:1")));
        assert_eq!(value.validate(&attempt, Some(&receipt)), Ok(()));

        value.compensation_attempt = Some(ExecutionAttemptRef(id("attempt:1")));
        assert_eq!(
            value.validate(&attempt, Some(&receipt)),
            Err(ReconciliationError::SelfCompensation)
        );
    }

    #[test]
    fn outcomes_bind_back_to_the_same_decision_and_reconciliation() {
        let attempt = attempt();
        let unknown = reconciliation(EconomicEffectDisposition::StillUnknown);
        let mut outcome = DecisionOutcomeRecord {
            decision: decision_ref(),
            reconciliation: ReconciliationRef(id("reconciliation:1")),
            outcome_observations: BTreeSet::from([observation("observation:outcome:1")]),
            observed_at_unix_ms: 2_000,
            evaluation_horizon_ms: 86_400_000,
            calibration_evidence: Some(id("calibration:1")),
        };
        assert_eq!(
            outcome.validate(&unknown, &attempt),
            Err(DecisionOutcomeError::EconomicEffectStillUnknown)
        );

        let applied = reconciliation(EconomicEffectDisposition::Applied);
        assert_eq!(outcome.validate(&applied, &attempt), Ok(()));

        outcome.decision = DecisionCapsuleRef {
            id: id("decision:other"),
            digest: Digest32::repeat(90),
        };
        assert_eq!(
            outcome.validate(&applied, &attempt),
            Err(DecisionOutcomeError::DecisionMismatch)
        );
    }
}

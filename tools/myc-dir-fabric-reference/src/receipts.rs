//! Canonical authorization-decision and effect-observation receipt contracts.
//!
//! A decision receipt says what an evaluator decided for one exact request.
//! An effect-observation receipt says what an independently observed effect
//! owner reports for one logical operation/attempt. These records are
//! intentionally separate: an Allow does not prove an effect occurred.
//!
//! This module creates canonical bytes and checks local binding semantics. It
//! does not hash, sign, verify signatures, establish observer independence, or
//! prove that an effect owner inventory is complete.

use super::causal_cut::CausalCutAssessment;
use super::{append_identifier, Digest, RealmId, MAX_IDENTIFIER_BYTES};

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct ReceiptId(String);

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct ClockProfileId(String);

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct LogicalOperationId(String);

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct AttemptId(String);

#[derive(Clone, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub struct EffectOwnerId(String);

fn checked_identifier(value: impl Into<String>) -> Result<String, ReceiptError> {
    let value = value.into();
    if value.trim().is_empty() {
        return Err(ReceiptError::EmptyIdentifier);
    }
    if value.len() > MAX_IDENTIFIER_BYTES {
        return Err(ReceiptError::IdentifierTooLong);
    }
    Ok(value)
}

macro_rules! impl_identifier {
    ($name:ident) => {
        impl $name {
            pub fn new(value: impl Into<String>) -> Result<Self, ReceiptError> {
                checked_identifier(value).map(Self)
            }

            pub fn as_str(&self) -> &str {
                &self.0
            }
        }
    };
}

impl_identifier!(ReceiptId);
impl_identifier!(ClockProfileId);
impl_identifier!(LogicalOperationId);
impl_identifier!(AttemptId);
impl_identifier!(EffectOwnerId);

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ReceiptError {
    EmptyIdentifier,
    IdentifierTooLong,
    ZeroDigest,
    InvalidTimeRange,
    PermitWithoutSufficientCut,
    MissingDecisionBasis,
    RequestBindingMismatch,
    DecisionReceiptBindingMismatch,
    EffectCannotCommitWithoutAllow,
    InvalidAttemptNumber,
    EmptyEffectHistory,
    EffectIdentityMismatch,
    DuplicateAttemptId,
    DuplicateAttemptNumber,
    AttemptSequenceGap,
    DuplicateCommit,
    AttemptAfterCommit,
    ContradictoryEffectObservations,
    CanonicalEncodingFailure,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum DecisionOutcome {
    Allow = 1,
    Deny = 2,
    Indeterminate = 3,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum DecisionBasisKind {
    ExactPermitWitness = 1,
    ExplicitForbidWitness = 2,
    CompletePolicySetDefaultDeny = 3,
    IncompleteAuthorityDependencies = 4,
    PolicyEvaluationIndeterminate = 5,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum PolicyEvaluation {
    /// One exact permit witness supplied by the existing full-context
    /// authorization evaluator (#4743/#4752 and the selected policy engine).
    Permit { witness_digest: Digest },
    /// A known explicit forbid. A known forbid remains a safe denial even if
    /// another authority dependency is unavailable.
    Forbid { forbid_digest: Digest },
    /// No permit matched in an independently complete policy/evidence set.
    DefaultDeny { evaluated_set_digest: Digest },
    /// A required policy/evidence evaluation could not be completed.
    Indeterminate { cause_digest: Digest },
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DecisionReceipt {
    id: ReceiptId,
    realm: RealmId,
    request_digest: Digest,
    policy_digest: Digest,
    cut_digest: Digest,
    closure_receipt_digest: Digest,
    snapshot_digest: Digest,
    evaluator_build_digest: Digest,
    nonce_digest: Digest,
    clock_profile: ClockProfileId,
    issued_at_ms: u64,
    expires_at_ms: u64,
    cut_assessment: CausalCutAssessment,
    outcome: DecisionOutcome,
    basis_kind: DecisionBasisKind,
    basis_digest: Digest,
}

impl DecisionReceipt {
    #[allow(clippy::too_many_arguments)]
    pub fn compose(
        id: ReceiptId,
        realm: RealmId,
        request_digest: Digest,
        policy_digest: Digest,
        cut_digest: Digest,
        closure_receipt_digest: Digest,
        snapshot_digest: Digest,
        evaluator_build_digest: Digest,
        nonce_digest: Digest,
        clock_profile: ClockProfileId,
        issued_at_ms: u64,
        expires_at_ms: u64,
        cut_assessment: CausalCutAssessment,
        policy_evaluation: PolicyEvaluation,
    ) -> Result<Self, ReceiptError> {
        if issued_at_ms >= expires_at_ms {
            return Err(ReceiptError::InvalidTimeRange);
        }
        for digest in [
            request_digest,
            policy_digest,
            cut_digest,
            closure_receipt_digest,
            snapshot_digest,
            evaluator_build_digest,
            nonce_digest,
        ] {
            if digest == [0; 32] {
                return Err(ReceiptError::ZeroDigest);
            }
        }

        let (outcome, basis_kind, basis_digest) = match policy_evaluation {
            PolicyEvaluation::Permit { witness_digest } => {
                if witness_digest == [0; 32] {
                    return Err(ReceiptError::MissingDecisionBasis);
                }
                if cut_assessment == CausalCutAssessment::Sufficient {
                    (
                        DecisionOutcome::Allow,
                        DecisionBasisKind::ExactPermitWitness,
                        witness_digest,
                    )
                } else {
                    (
                        DecisionOutcome::Indeterminate,
                        DecisionBasisKind::IncompleteAuthorityDependencies,
                        cut_digest,
                    )
                }
            }
            PolicyEvaluation::Forbid { forbid_digest } => {
                if forbid_digest == [0; 32] {
                    return Err(ReceiptError::MissingDecisionBasis);
                }
                (
                    DecisionOutcome::Deny,
                    DecisionBasisKind::ExplicitForbidWitness,
                    forbid_digest,
                )
            }
            PolicyEvaluation::DefaultDeny {
                evaluated_set_digest,
            } => {
                if evaluated_set_digest == [0; 32] {
                    return Err(ReceiptError::MissingDecisionBasis);
                }
                if cut_assessment == CausalCutAssessment::Sufficient {
                    (
                        DecisionOutcome::Deny,
                        DecisionBasisKind::CompletePolicySetDefaultDeny,
                        evaluated_set_digest,
                    )
                } else {
                    (
                        DecisionOutcome::Indeterminate,
                        DecisionBasisKind::IncompleteAuthorityDependencies,
                        cut_digest,
                    )
                }
            }
            PolicyEvaluation::Indeterminate { cause_digest } => {
                if cause_digest == [0; 32] {
                    return Err(ReceiptError::MissingDecisionBasis);
                }
                (
                    DecisionOutcome::Indeterminate,
                    DecisionBasisKind::PolicyEvaluationIndeterminate,
                    cause_digest,
                )
            }
        };

        Ok(Self {
            id,
            realm,
            request_digest,
            policy_digest,
            cut_digest,
            closure_receipt_digest,
            snapshot_digest,
            evaluator_build_digest,
            nonce_digest,
            clock_profile,
            issued_at_ms,
            expires_at_ms,
            cut_assessment,
            outcome,
            basis_kind,
            basis_digest,
        })
    }

    pub fn id(&self) -> &ReceiptId {
        &self.id
    }

    pub fn realm(&self) -> &RealmId {
        &self.realm
    }

    pub fn request_digest(&self) -> &Digest {
        &self.request_digest
    }

    pub fn outcome(&self) -> DecisionOutcome {
        self.outcome
    }

    pub fn clock_profile_id(&self) -> &str {
        self.clock_profile.as_str()
    }

    pub fn issued_at_ms(&self) -> u64 {
        self.issued_at_ms
    }

    pub fn expires_at_ms(&self) -> u64 {
        self.expires_at_ms
    }

    pub fn cut_assessment(&self) -> CausalCutAssessment {
        self.cut_assessment
    }

    /// Deterministic bytes to be signed by the evaluator's pinned identity.
    /// These bytes alone do not authenticate the evaluator or decision truth.
    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ReceiptError> {
        let mut bytes = b"MYC-DIR-AUTHORIZATION-DECISION-RECEIPT\0\x01".to_vec();
        append_identifier(&mut bytes, self.id.as_str())
            .map_err(|_| ReceiptError::IdentifierTooLong)?;
        append_identifier(&mut bytes, self.realm.as_str())
            .map_err(|_| ReceiptError::IdentifierTooLong)?;
        bytes.extend_from_slice(&self.request_digest);
        bytes.extend_from_slice(&self.policy_digest);
        bytes.extend_from_slice(&self.cut_digest);
        bytes.extend_from_slice(&self.closure_receipt_digest);
        bytes.extend_from_slice(&self.snapshot_digest);
        bytes.extend_from_slice(&self.evaluator_build_digest);
        bytes.extend_from_slice(&self.nonce_digest);
        append_identifier(&mut bytes, self.clock_profile.as_str())
            .map_err(|_| ReceiptError::IdentifierTooLong)?;
        bytes.extend_from_slice(&self.issued_at_ms.to_be_bytes());
        bytes.extend_from_slice(&self.expires_at_ms.to_be_bytes());
        bytes.push(cut_assessment_tag(self.cut_assessment));
        bytes.push(self.outcome as u8);
        bytes.push(self.basis_kind as u8);
        bytes.extend_from_slice(&self.basis_digest);
        Ok(bytes)
    }
}

fn cut_assessment_tag(assessment: CausalCutAssessment) -> u8 {
    match assessment {
        CausalCutAssessment::Sufficient => 1,
        CausalCutAssessment::ClosureUnverified => 2,
        CausalCutAssessment::ClosureIncomplete => 3,
        CausalCutAssessment::ClosureRejected => 4,
        CausalCutAssessment::ClosureUnavailable => 5,
        CausalCutAssessment::RequestBindingMismatch => 6,
        CausalCutAssessment::RealmBindingMismatch => 7,
        CausalCutAssessment::PolicyBindingMismatch => 8,
        CausalCutAssessment::DependencyGraphBindingMismatch => 9,
        CausalCutAssessment::ResolverProfileMismatch => 10,
        CausalCutAssessment::CutDigestMismatch => 11,
        CausalCutAssessment::BehindRequiredDependency => 12,
        CausalCutAssessment::IncomparableRequiredDependency => 13,
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum EffectOutcome {
    Committed = 1,
    RejectedNoEffect = 2,
    Unknown = 3,
    NotObserved = 4,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum ReconciliationOutcome {
    ConfirmedCommitted = 1,
    ConfirmedNoEffect = 2,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EffectObservationReceipt {
    request_digest: Digest,
    decision_receipt_digest: Digest,
    logical_operation_id: LogicalOperationId,
    attempt_id: AttemptId,
    attempt_number: u32,
    idempotency_key_digest: Digest,
    effect_owner: EffectOwnerId,
    outcome: EffectOutcome,
    observation_digest: Digest,
}

impl EffectObservationReceipt {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        request_digest: Digest,
        decision_receipt_digest: Digest,
        logical_operation_id: LogicalOperationId,
        attempt_id: AttemptId,
        attempt_number: u32,
        idempotency_key_digest: Digest,
        effect_owner: EffectOwnerId,
        outcome: EffectOutcome,
        observation_digest: Digest,
    ) -> Result<Self, ReceiptError> {
        if attempt_number == 0 {
            return Err(ReceiptError::InvalidAttemptNumber);
        }
        if request_digest == [0; 32]
            || decision_receipt_digest == [0; 32]
            || idempotency_key_digest == [0; 32]
            || observation_digest == [0; 32]
        {
            return Err(ReceiptError::ZeroDigest);
        }
        Ok(Self {
            request_digest,
            decision_receipt_digest,
            logical_operation_id,
            attempt_id,
            attempt_number,
            idempotency_key_digest,
            effect_owner,
            outcome,
            observation_digest,
        })
    }

    pub fn outcome(&self) -> EffectOutcome {
        self.outcome
    }

    pub fn attempt_id(&self) -> &AttemptId {
        &self.attempt_id
    }

    pub fn attempt_number(&self) -> u32 {
        self.attempt_number
    }

    pub fn logical_operation_id(&self) -> &LogicalOperationId {
        &self.logical_operation_id
    }

    /// Confirms the exact decision/request binding. It does not authenticate
    /// the effect owner or prove that its observation inventory is complete.
    pub fn validate_against_decision(
        &self,
        decision: &DecisionReceipt,
        expected_decision_receipt_digest: &Digest,
    ) -> Result<(), ReceiptError> {
        if &self.request_digest != decision.request_digest() {
            return Err(ReceiptError::RequestBindingMismatch);
        }
        if &self.decision_receipt_digest != expected_decision_receipt_digest {
            return Err(ReceiptError::DecisionReceiptBindingMismatch);
        }
        if self.outcome == EffectOutcome::Committed
            && decision.outcome() != DecisionOutcome::Allow
        {
            return Err(ReceiptError::EffectCannotCommitWithoutAllow);
        }
        Ok(())
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ReceiptError> {
        let mut bytes = b"MYC-DIR-EFFECT-ATTEMPT-RECEIPT\0\x01".to_vec();
        bytes.extend_from_slice(&self.request_digest);
        bytes.extend_from_slice(&self.decision_receipt_digest);
        append_identifier(&mut bytes, self.logical_operation_id.as_str())
            .map_err(|_| ReceiptError::IdentifierTooLong)?;
        append_identifier(&mut bytes, self.attempt_id.as_str())
            .map_err(|_| ReceiptError::IdentifierTooLong)?;
        bytes.extend_from_slice(&self.attempt_number.to_be_bytes());
        bytes.extend_from_slice(&self.idempotency_key_digest);
        append_identifier(&mut bytes, self.effect_owner.as_str())
            .map_err(|_| ReceiptError::IdentifierTooLong)?;
        bytes.push(self.outcome as u8);
        bytes.extend_from_slice(&self.observation_digest);
        Ok(bytes)
    }
}

/// A reconciliation is a separate observation, not a new physical attempt.
/// It points to the exact previous attempt whose outcome was Unknown/NotObserved.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct EffectReconciliationReceipt {
    request_digest: Digest,
    decision_receipt_digest: Digest,
    logical_operation_id: LogicalOperationId,
    target_attempt_id: AttemptId,
    target_attempt_number: u32,
    idempotency_key_digest: Digest,
    effect_owner: EffectOwnerId,
    outcome: ReconciliationOutcome,
    observation_digest: Digest,
}

impl EffectReconciliationReceipt {
    #[allow(clippy::too_many_arguments)]
    pub fn new(
        request_digest: Digest,
        decision_receipt_digest: Digest,
        logical_operation_id: LogicalOperationId,
        target_attempt_id: AttemptId,
        target_attempt_number: u32,
        idempotency_key_digest: Digest,
        effect_owner: EffectOwnerId,
        outcome: ReconciliationOutcome,
        observation_digest: Digest,
    ) -> Result<Self, ReceiptError> {
        if target_attempt_number == 0 {
            return Err(ReceiptError::InvalidAttemptNumber);
        }
        if request_digest == [0; 32]
            || decision_receipt_digest == [0; 32]
            || idempotency_key_digest == [0; 32]
            || observation_digest == [0; 32]
        {
            return Err(ReceiptError::ZeroDigest);
        }
        Ok(Self {
            request_digest,
            decision_receipt_digest,
            logical_operation_id,
            target_attempt_id,
            target_attempt_number,
            idempotency_key_digest,
            effect_owner,
            outcome,
            observation_digest,
        })
    }

    pub fn target_attempt_id(&self) -> &AttemptId {
        &self.target_attempt_id
    }

    pub fn outcome(&self) -> ReconciliationOutcome {
        self.outcome
    }

    pub fn validate_against_decision(
        &self,
        decision: &DecisionReceipt,
        expected_decision_receipt_digest: &Digest,
    ) -> Result<(), ReceiptError> {
        if &self.request_digest != decision.request_digest() {
            return Err(ReceiptError::RequestBindingMismatch);
        }
        if &self.decision_receipt_digest != expected_decision_receipt_digest {
            return Err(ReceiptError::DecisionReceiptBindingMismatch);
        }
        if self.outcome == ReconciliationOutcome::ConfirmedCommitted
            && decision.outcome() != DecisionOutcome::Allow
        {
            return Err(ReceiptError::EffectCannotCommitWithoutAllow);
        }
        Ok(())
    }

    pub fn canonical_bytes(&self) -> Result<Vec<u8>, ReceiptError> {
        let mut bytes = b"MYC-DIR-EFFECT-RECONCILIATION-RECEIPT\0\x01".to_vec();
        bytes.extend_from_slice(&self.request_digest);
        bytes.extend_from_slice(&self.decision_receipt_digest);
        append_identifier(&mut bytes, self.logical_operation_id.as_str())
            .map_err(|_| ReceiptError::IdentifierTooLong)?;
        append_identifier(&mut bytes, self.target_attempt_id.as_str())
            .map_err(|_| ReceiptError::IdentifierTooLong)?;
        bytes.extend_from_slice(&self.target_attempt_number.to_be_bytes());
        bytes.extend_from_slice(&self.idempotency_key_digest);
        append_identifier(&mut bytes, self.effect_owner.as_str())
            .map_err(|_| ReceiptError::IdentifierTooLong)?;
        bytes.push(self.outcome as u8);
        bytes.extend_from_slice(&self.observation_digest);
        Ok(bytes)
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum EffectHistoryAssessment {
    CommittedOnce,
    ReconciledNoEffect,
    RejectedOnly,
    UnresolvedUnknown,
    InvalidIdentityMismatch,
    InvalidDuplicateAttempt,
    InvalidAttemptSequence,
    InvalidDuplicateReconciliation,
    InvalidReconciliationTarget,
    InvalidReconciliationOfResolvedAttempt,
    InvalidAttemptWhileUnknown,
    InvalidDuplicateCommit,
    InvalidAttemptAfterCommit,
}

impl EffectHistoryAssessment {
    /// A new physical attempt is eligible only after a known no-effect result
    /// (directly observed or independently reconciled). Unknown is not no-effect.
    pub fn permits_new_attempt(self) -> bool {
        matches!(self, Self::RejectedOnly | Self::ReconciledNoEffect)
    }
}

/// Assesses a logical operation using physical attempt receipts and a separate
/// set of owner-side reconciliation receipts. Reconciliation never consumes an
/// attempt number. Every new physical attempt must follow a resolved no-effect
/// outcome; a prior Unknown cannot be bypassed by appending another attempt.
pub fn assess_effect_history(
    attempts: &[EffectObservationReceipt],
    reconciliations: &[EffectReconciliationReceipt],
) -> EffectHistoryAssessment {
    if attempts.is_empty() {
        return EffectHistoryAssessment::InvalidAttemptSequence;
    }

    let mut ordered = attempts.to_vec();
    ordered.sort_by_key(|receipt| receipt.attempt_number);
    let first = &ordered[0];

    for attempt in &ordered {
        if attempt.logical_operation_id != first.logical_operation_id
            || attempt.request_digest != first.request_digest
            || attempt.idempotency_key_digest != first.idempotency_key_digest
            || attempt.effect_owner != first.effect_owner
        {
            return EffectHistoryAssessment::InvalidIdentityMismatch;
        }
    }

    for pair in ordered.windows(2) {
        if pair[0].attempt_id == pair[1].attempt_id
            || pair[0].attempt_number == pair[1].attempt_number
        {
            return EffectHistoryAssessment::InvalidDuplicateAttempt;
        }
    }

    if ordered
        .iter()
        .enumerate()
        .any(|(index, receipt)| receipt.attempt_number != (index as u32) + 1)
    {
        return EffectHistoryAssessment::InvalidAttemptSequence;
    }

    let mut reconciliation_by_attempt: std::collections::BTreeMap<AttemptId, &EffectReconciliationReceipt> =
        std::collections::BTreeMap::new();

    for reconciliation in reconciliations {
        if reconciliation.logical_operation_id != first.logical_operation_id
            || reconciliation.request_digest != first.request_digest
            || reconciliation.idempotency_key_digest != first.idempotency_key_digest
            || reconciliation.effect_owner != first.effect_owner
        {
            return EffectHistoryAssessment::InvalidIdentityMismatch;
        }

        let Some(target) = ordered
            .iter()
            .find(|attempt| attempt.attempt_id == reconciliation.target_attempt_id)
        else {
            return EffectHistoryAssessment::InvalidReconciliationTarget;
        };

        if target.attempt_number != reconciliation.target_attempt_number
            || target.decision_receipt_digest != reconciliation.decision_receipt_digest
        {
            return EffectHistoryAssessment::InvalidReconciliationTarget;
        }

        if !matches!(target.outcome, EffectOutcome::Unknown | EffectOutcome::NotObserved) {
            return EffectHistoryAssessment::InvalidReconciliationOfResolvedAttempt;
        }

        if reconciliation_by_attempt
            .insert(reconciliation.target_attempt_id.clone(), reconciliation)
            .is_some()
        {
            return EffectHistoryAssessment::InvalidDuplicateReconciliation;
        }
    }

    let mut commits = 0usize;
    let mut committed_at: Option<usize> = None;
    let mut any_reconciled_no_effect = false;
    let mut any_rejected_no_effect = false;
    let mut last_unresolved = false;

    for (index, attempt) in ordered.iter().enumerate() {
        let reconciliation = reconciliation_by_attempt.get(&attempt.attempt_id).copied();

        let effective = match (attempt.outcome, reconciliation.map(|r| r.outcome)) {
            (EffectOutcome::Committed, None) => EffectiveEffectOutcome::Committed,
            (EffectOutcome::RejectedNoEffect, None) => EffectiveEffectOutcome::NoEffect,
            (EffectOutcome::Unknown | EffectOutcome::NotObserved, None) => {
                EffectiveEffectOutcome::Unknown
            }
            (
                EffectOutcome::Unknown | EffectOutcome::NotObserved,
                Some(ReconciliationOutcome::ConfirmedCommitted),
            ) => EffectiveEffectOutcome::Committed,
            (
                EffectOutcome::Unknown | EffectOutcome::NotObserved,
                Some(ReconciliationOutcome::ConfirmedNoEffect),
            ) => EffectiveEffectOutcome::ReconciledNoEffect,
            _ => return EffectHistoryAssessment::InvalidReconciliationOfResolvedAttempt,
        };

        match effective {
            EffectiveEffectOutcome::Committed => {
                commits += 1;
                committed_at = Some(index);
                if commits > 1 {
                    return EffectHistoryAssessment::InvalidDuplicateCommit;
                }
                last_unresolved = false;
            }
            EffectiveEffectOutcome::NoEffect => {
                any_rejected_no_effect = true;
                last_unresolved = false;
            }
            EffectiveEffectOutcome::ReconciledNoEffect => {
                any_reconciled_no_effect = true;
                last_unresolved = false;
            }
            EffectiveEffectOutcome::Unknown => {
                if index + 1 != ordered.len() {
                    return EffectHistoryAssessment::InvalidAttemptWhileUnknown;
                }
                last_unresolved = true;
            }
        }
    }

    if commits == 1 {
        if committed_at.is_some_and(|index| index + 1 != ordered.len()) {
            return EffectHistoryAssessment::InvalidAttemptAfterCommit;
        }
        EffectHistoryAssessment::CommittedOnce
    } else if last_unresolved {
        EffectHistoryAssessment::UnresolvedUnknown
    } else if any_reconciled_no_effect {
        EffectHistoryAssessment::ReconciledNoEffect
    } else if any_rejected_no_effect {
        EffectHistoryAssessment::RejectedOnly
    } else {
        EffectHistoryAssessment::UnresolvedUnknown
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum EffectiveEffectOutcome {
    Committed,
    NoEffect,
    ReconciledNoEffect,
    Unknown,
}

#[cfg(test)]
mod tests {
    use super::*;

    fn id<T>(
        value: &str,
        constructor: impl FnOnce(String) -> Result<T, ReceiptError>,
    ) -> T {
        constructor(value.to_owned()).unwrap()
    }

    fn receipt(
        cut_assessment: CausalCutAssessment,
        evaluation: PolicyEvaluation,
    ) -> DecisionReceipt {
        DecisionReceipt::compose(
            id("decision-1", ReceiptId::new),
            RealmId::new("realm-a").unwrap(),
            [1; 32],
            [2; 32],
            [3; 32],
            [13; 32],
            [4; 32],
            [5; 32],
            [6; 32],
            id("trusted-utc-v1", ClockProfileId::new),
            100,
            200,
            cut_assessment,
            evaluation,
        )
        .unwrap()
    }

    fn effect(
        attempt_number: u32,
        outcome: EffectOutcome,
        decision_digest: Digest,
    ) -> EffectObservationReceipt {
        EffectObservationReceipt::new(
            [1; 32],
            decision_digest,
            id("logical-op-1", LogicalOperationId::new),
            id(&format!("attempt-{attempt_number}"), AttemptId::new),
            attempt_number,
            [7; 32],
            id("resource-owner", EffectOwnerId::new),
            outcome,
            [8; 32],
        )
        .unwrap()
    }

    fn reconcile(
        target_attempt_number: u32,
        outcome: ReconciliationOutcome,
        decision_digest: Digest,
    ) -> EffectReconciliationReceipt {
        EffectReconciliationReceipt::new(
            [1; 32],
            decision_digest,
            id("logical-op-1", LogicalOperationId::new),
            id(&format!("attempt-{target_attempt_number}"), AttemptId::new),
            target_attempt_number,
            [7; 32],
            id("resource-owner", EffectOwnerId::new),
            outcome,
            [12; 32],
        )
        .unwrap()
    }

    #[test]
    fn complete_cut_and_exact_policy_witness_can_produce_allow() {
        let decision = receipt(
            CausalCutAssessment::Sufficient,
            PolicyEvaluation::Permit { witness_digest: [9; 32] },
        );
        assert_eq!(decision.outcome(), DecisionOutcome::Allow);
        assert_eq!(decision.basis_kind, DecisionBasisKind::ExactPermitWitness);
    }

    #[test]
    fn permit_with_incomplete_dependency_cut_becomes_indeterminate() {
        let decision = receipt(
            CausalCutAssessment::ClosureIncomplete,
            PolicyEvaluation::Permit { witness_digest: [9; 32] },
        );
        assert_eq!(decision.outcome(), DecisionOutcome::Indeterminate);
        assert_eq!(decision.basis_kind, DecisionBasisKind::IncompleteAuthorityDependencies);
    }

    #[test]
    fn known_explicit_forbid_remains_deny_even_when_another_dependency_is_missing() {
        let decision = receipt(
            CausalCutAssessment::ClosureIncomplete,
            PolicyEvaluation::Forbid { forbid_digest: [9; 32] },
        );
        assert_eq!(decision.outcome(), DecisionOutcome::Deny);
    }

    #[test]
    fn default_deny_requires_a_complete_policy_and_dependency_set() {
        let complete = receipt(
            CausalCutAssessment::Sufficient,
            PolicyEvaluation::DefaultDeny { evaluated_set_digest: [9; 32] },
        );
        let incomplete = receipt(
            CausalCutAssessment::ClosureUnavailable,
            PolicyEvaluation::DefaultDeny { evaluated_set_digest: [9; 32] },
        );
        assert_eq!(complete.outcome(), DecisionOutcome::Deny);
        assert_eq!(incomplete.outcome(), DecisionOutcome::Indeterminate);
    }

    #[test]
    fn canonical_decision_receipt_binds_security_relevant_inputs() {
        let base = receipt(
            CausalCutAssessment::Sufficient,
            PolicyEvaluation::Permit { witness_digest: [9; 32] },
        );
        let bytes = base.canonical_bytes().unwrap();
        let changed = DecisionReceipt::compose(
            id("decision-2", ReceiptId::new),
            base.realm.clone(),
            *base.request_digest(),
            [22; 32],
            *base.cut_digest,
            *base.closure_receipt_digest,
            *base.snapshot_digest,
            *base.evaluator_build_digest,
            *base.nonce_digest,
            base.clock_profile.clone(),
            base.issued_at_ms,
            base.expires_at_ms,
            base.cut_assessment,
            PolicyEvaluation::Permit { witness_digest: [9; 32] },
        )
        .unwrap();
        assert_ne!(bytes, changed.canonical_bytes().unwrap());
    }

    #[test]
    fn committed_effect_requires_exact_allow_receipt_binding() {
        let allow = receipt(
            CausalCutAssessment::Sufficient,
            PolicyEvaluation::Permit { witness_digest: [9; 32] },
        );
        let committed = effect(1, EffectOutcome::Committed, [10; 32]);
        assert_eq!(
            committed.validate_against_decision(&allow, &[11; 32]),
            Err(ReceiptError::DecisionReceiptBindingMismatch)
        );

        let non_allow = receipt(
            CausalCutAssessment::ClosureIncomplete,
            PolicyEvaluation::Permit { witness_digest: [9; 32] },
        );
        let invalid = effect(1, EffectOutcome::Committed, [11; 32]);
        assert_eq!(
            invalid.validate_against_decision(&non_allow, &[11; 32]),
            Err(ReceiptError::EffectCannotCommitWithoutAllow)
        );
    }

    #[test]
    fn unknown_effect_outcome_blocks_new_attempt_until_reconciled() {
        let unknown = effect(1, EffectOutcome::Unknown, [10; 32]);
        let unresolved = assess_effect_history(&[unknown.clone()], &[]);
        assert_eq!(unresolved, EffectHistoryAssessment::UnresolvedUnknown);
        assert!(!unresolved.permits_new_attempt());

        let reconciled = reconcile(1, ReconciliationOutcome::ConfirmedNoEffect, [10; 32]);
        let resolved = assess_effect_history(&[unknown], &[reconciled]);
        assert_eq!(resolved, EffectHistoryAssessment::ReconciledNoEffect);
        assert!(resolved.permits_new_attempt());
    }

    #[test]
    fn retry_after_no_effect_reconciliation_can_become_unknown_again() {
        let first_unknown = effect(1, EffectOutcome::Unknown, [10; 32]);
        let retry_unknown = effect(2, EffectOutcome::Unknown, [11; 32]);
        let reconciliation = reconcile(1, ReconciliationOutcome::ConfirmedNoEffect, [10; 32]);
        let status = assess_effect_history(&[first_unknown, retry_unknown], &[reconciliation]);
        assert_eq!(status, EffectHistoryAssessment::UnresolvedUnknown);
        assert!(!status.permits_new_attempt());
    }

    #[test]
    fn a_new_attempt_without_reconciling_the_prior_unknown_is_rejected() {
        let first = effect(1, EffectOutcome::Unknown, [10; 32]);
        let second = effect(2, EffectOutcome::RejectedNoEffect, [11; 32]);
        assert_eq!(
            assess_effect_history(&[first, second], &[]),
            EffectHistoryAssessment::InvalidAttemptWhileUnknown
        );
    }

    #[test]
    fn retry_must_preserve_logical_operation_and_idempotency_commitment() {
        let first = effect(1, EffectOutcome::Unknown, [10; 32]);
        let second = EffectObservationReceipt::new(
            [1; 32],
            [11; 32],
            id("logical-op-different", LogicalOperationId::new),
            id("attempt-2", AttemptId::new),
            2,
            [7; 32],
            id("resource-owner", EffectOwnerId::new),
            EffectOutcome::RejectedNoEffect,
            [12; 32],
        )
        .unwrap();
        assert_eq!(
            assess_effect_history(&[first, second], &[]),
            EffectHistoryAssessment::InvalidIdentityMismatch
        );
    }

    #[test]
    fn second_committed_physical_attempt_is_rejected() {
        let first = effect(1, EffectOutcome::Committed, [10; 32]);
        let second = effect(2, EffectOutcome::Committed, [11; 32]);
        assert_eq!(
            assess_effect_history(&[first, second], &[]),
            EffectHistoryAssessment::InvalidDuplicateCommit
        );
    }

    #[test]
    fn owner_reconciliation_of_commit_plus_later_commit_is_duplicate_effect() {
        let first_unknown = effect(1, EffectOutcome::Unknown, [10; 32]);
        let second_commit = effect(2, EffectOutcome::Committed, [11; 32]);
        let first_reconciled_committed =
            reconcile(1, ReconciliationOutcome::ConfirmedCommitted, [10; 32]);
        assert_eq!(
            assess_effect_history(
                &[first_unknown, second_commit],
                &[first_reconciled_committed]
            ),
            EffectHistoryAssessment::InvalidDuplicateCommit
        );
    }

    #[test]
    fn reconciliation_must_target_an_exact_unknown_attempt() {
        let unknown = effect(1, EffectOutcome::Unknown, [10; 32]);
        let wrong_target = EffectReconciliationReceipt::new(
            [1; 32],
            [10; 32],
            id("logical-op-1", LogicalOperationId::new),
            id("missing-attempt", AttemptId::new),
            1,
            [7; 32],
            id("resource-owner", EffectOwnerId::new),
            ReconciliationOutcome::ConfirmedNoEffect,
            [12; 32],
        )
        .unwrap();
        assert_eq!(
            assess_effect_history(&[unknown], &[wrong_target]),
            EffectHistoryAssessment::InvalidReconciliationTarget
        );
    }

    #[test]
    fn reconciliation_does_not_consume_a_physical_attempt_number() {
        let first_unknown = effect(1, EffectOutcome::Unknown, [10; 32]);
        let second_rejected = effect(2, EffectOutcome::RejectedNoEffect, [11; 32]);
        let confirmed_no_effect =
            reconcile(1, ReconciliationOutcome::ConfirmedNoEffect, [10; 32]);
        let status = assess_effect_history(
            &[first_unknown, second_rejected],
            &[confirmed_no_effect],
        );
        assert_eq!(status, EffectHistoryAssessment::ReconciledNoEffect);
        assert!(status.permits_new_attempt());
    }

    #[test]
    fn duplicate_reconciliation_for_one_attempt_is_rejected() {
        let unknown = effect(1, EffectOutcome::Unknown, [10; 32]);
        let first = reconcile(1, ReconciliationOutcome::ConfirmedNoEffect, [10; 32]);
        let second = reconcile(1, ReconciliationOutcome::ConfirmedNoEffect, [10; 32]);
        assert_eq!(
            assess_effect_history(&[unknown], &[first, second]),
            EffectHistoryAssessment::InvalidDuplicateReconciliation
        );
    }

    #[test]
    fn reconciliation_cannot_rewrite_a_resolved_attempt() {
        let rejected = effect(1, EffectOutcome::RejectedNoEffect, [10; 32]);
        let forged = reconcile(1, ReconciliationOutcome::ConfirmedCommitted, [10; 32]);
        assert_eq!(
            assess_effect_history(&[rejected], &[forged]),
            EffectHistoryAssessment::InvalidReconciliationOfResolvedAttempt
        );
    }

    #[test]
    fn reconciliation_confirmed_commit_closes_the_logical_operation() {
        let unknown = effect(1, EffectOutcome::Unknown, [10; 32]);
        let confirmed_commit =
            reconcile(1, ReconciliationOutcome::ConfirmedCommitted, [10; 32]);
        assert_eq!(
            assess_effect_history(&[unknown], &[confirmed_commit]),
            EffectHistoryAssessment::CommittedOnce
        );
        assert!(!EffectHistoryAssessment::CommittedOnce.permits_new_attempt());
    }

    #[test]
    fn confirmed_commit_reconciliation_requires_allow_decision_receipt() {
        let denied = receipt(
            CausalCutAssessment::Sufficient,
            PolicyEvaluation::DefaultDeny { evaluated_set_digest: [9; 32] },
        );
        let reconciliation = reconcile(
            1,
            ReconciliationOutcome::ConfirmedCommitted,
            [10; 32],
        );
        assert_eq!(
            reconciliation.validate_against_decision(&denied, &[10; 32]),
            Err(ReceiptError::EffectCannotCommitWithoutAllow)
        );
    }

    #[test]
    fn reconciliation_decision_mismatch_is_rejected() {
        let unknown = effect(1, EffectOutcome::Unknown, [10; 32]);
        let wrong_decision = reconcile(1, ReconciliationOutcome::ConfirmedNoEffect, [99; 32]);
        assert_eq!(
            assess_effect_history(&[unknown], &[wrong_decision]),
            EffectHistoryAssessment::InvalidReconciliationTarget
        );
    }

    #[test]
    fn successful_effect_history_is_distinct_from_authorization() {
        let decision = receipt(
            CausalCutAssessment::Sufficient,
            PolicyEvaluation::Permit { witness_digest: [9; 32] },
        );
        assert_eq!(decision.outcome(), DecisionOutcome::Allow);
        let observation = effect(1, EffectOutcome::Committed, [10; 32]);
        observation.validate_against_decision(&decision, &[10; 32]).unwrap();
        assert_eq!(
            assess_effect_history(&[observation], &[]),
            EffectHistoryAssessment::CommittedOnce
        );
        assert!(!EffectHistoryAssessment::CommittedOnce.permits_new_attempt());
    }

    #[test]
    fn effect_history_rejects_attempt_number_gaps() {
        let first = effect(1, EffectOutcome::RejectedNoEffect, [10; 32]);
        let third = effect(3, EffectOutcome::RejectedNoEffect, [12; 32]);
        assert_eq!(
            assess_effect_history(&[first, third], &[]),
            EffectHistoryAssessment::InvalidAttemptSequence
        );
    }
}

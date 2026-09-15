use mycelix_business_core::{Digest32, ExecutionAttemptRef, ReferenceId, ReservationId};
use mycelix_finance_exact::AssetAmount;
use mycelix_finance_settlement::QualifiedSettlement;

use crate::FinanceBoundForAttempt;

/// Conservative Finance-side outcome that has not yet been reconciled by the
/// Business Fabric.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FinanceProjectionState {
    /// A sealed FIN-ECO-002 settlement proof matches the exact bound attempt.
    QualifiedSettlement {
        settlement_subject: ReferenceId,
        evidence_frontier: Digest32,
        qualified_at_unix_ms: u64,
    },
    /// Finance cannot yet establish the financial effect.
    Unknown {
        observation: ReferenceId,
        evidence_frontier: Digest32,
        observed_at_unix_ms: u64,
    },
}

/// Explicit unknown Finance outcome. Constructing this input is safe because it
/// can only preserve uncertainty; it cannot promote an effect to success.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct UnknownFinanceOutcome {
    pub attempt: ExecutionAttemptRef,
    pub observation: ReferenceId,
    pub evidence_frontier: Digest32,
    pub observed_at_unix_ms: u64,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum FinanceProjectionError {
    SettlementAttemptMismatch,
    SettlementAmountMismatch,
    FinalityProfileMismatch,
    SettlementPredatesBinding,
    ProjectionPredatesSettlement,
    UnknownAttemptMismatch,
    UnknownPredatesBinding,
    ProjectionPredatesUnknown,
}

/// Sealed Finance-domain input to later Business reconciliation.
///
/// This value deliberately does not contain a Business
/// `EconomicEffectDisposition`; Finance cannot self-assert commercial success.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct FinanceReconciliationProjection {
    attempt: ExecutionAttemptRef,
    reservation_id: ReservationId,
    amount: AssetAmount,
    finance_frontier: Digest32,
    state: FinanceProjectionState,
    projected_at_unix_ms: u64,
}

impl FinanceReconciliationProjection {
    fn new(
        bound: &FinanceBoundForAttempt,
        state: FinanceProjectionState,
        projected_at_unix_ms: u64,
    ) -> Self {
        Self {
            attempt: bound.attempt().clone(),
            reservation_id: bound.reservation_id().clone(),
            amount: bound.amount().clone(),
            finance_frontier: bound.finance_frontier(),
            state,
            projected_at_unix_ms,
        }
    }

    pub fn attempt(&self) -> &ExecutionAttemptRef {
        &self.attempt
    }

    pub fn reservation_id(&self) -> &ReservationId {
        &self.reservation_id
    }

    pub fn amount(&self) -> &AssetAmount {
        &self.amount
    }

    pub fn finance_frontier(&self) -> Digest32 {
        self.finance_frontier
    }

    pub fn state(&self) -> &FinanceProjectionState {
        &self.state
    }

    pub fn projected_at_unix_ms(&self) -> u64 {
        self.projected_at_unix_ms
    }
}

/// Project a sealed FIN-ECO-002 settlement proof into a Finance-side
/// reconciliation input. This still does not establish Business success or legal
/// discharge.
pub fn project_qualified_settlement(
    bound: &FinanceBoundForAttempt,
    settlement: &QualifiedSettlement,
    projected_at_unix_ms: u64,
) -> Result<FinanceReconciliationProjection, FinanceProjectionError> {
    if settlement.attempt() != bound.attempt() {
        return Err(FinanceProjectionError::SettlementAttemptMismatch);
    }
    if settlement.amount() != bound.amount() {
        return Err(FinanceProjectionError::SettlementAmountMismatch);
    }
    if settlement.profile() != bound.required_finality_profile() {
        return Err(FinanceProjectionError::FinalityProfileMismatch);
    }
    if settlement.qualified_at_unix_ms() < bound.bound_at_unix_ms() {
        return Err(FinanceProjectionError::SettlementPredatesBinding);
    }
    if projected_at_unix_ms < settlement.qualified_at_unix_ms() {
        return Err(FinanceProjectionError::ProjectionPredatesSettlement);
    }

    Ok(FinanceReconciliationProjection::new(
        bound,
        FinanceProjectionState::QualifiedSettlement {
            settlement_subject: settlement.subject().clone(),
            evidence_frontier: settlement.evidence_frontier(),
            qualified_at_unix_ms: settlement.qualified_at_unix_ms(),
        },
        projected_at_unix_ms,
    ))
}

/// Preserve an unresolved Finance outcome for later reconciliation. Unknown is a
/// first-class state and cannot be silently converted into success or failure.
pub fn project_unknown_finance_outcome(
    bound: &FinanceBoundForAttempt,
    unknown: &UnknownFinanceOutcome,
    projected_at_unix_ms: u64,
) -> Result<FinanceReconciliationProjection, FinanceProjectionError> {
    if &unknown.attempt != bound.attempt() {
        return Err(FinanceProjectionError::UnknownAttemptMismatch);
    }
    if unknown.observed_at_unix_ms < bound.bound_at_unix_ms() {
        return Err(FinanceProjectionError::UnknownPredatesBinding);
    }
    if projected_at_unix_ms < unknown.observed_at_unix_ms {
        return Err(FinanceProjectionError::ProjectionPredatesUnknown);
    }

    Ok(FinanceReconciliationProjection::new(
        bound,
        FinanceProjectionState::Unknown {
            observation: unknown.observation.clone(),
            evidence_frontier: unknown.evidence_frontier,
            observed_at_unix_ms: unknown.observed_at_unix_ms,
        },
        projected_at_unix_ms,
    ))
}

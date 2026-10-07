//! Professional banking/finance control-plane primitives.
//!
//! These types are deliberately dependency-light and contain no HDK, RPC,
//! accounting-engine, or regulatory-jurisdiction implementation. They define
//! a common evidence contract for event-sourced professional finance.

use serde::{Deserialize, Serialize};
use std::collections::BTreeMap;
use std::fmt;

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum PostingSide {
    Debit,
    Credit,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LedgerPosting {
    pub account_id: String,
    pub currency: String,
    pub amount_minor_units: u128,
    pub side: PostingSide,
    pub legal_entity_id: String,
    pub book: String,
}

impl LedgerPosting {
    pub fn validate(&self) -> Result<(), ProfessionalFinanceError> {
        require_nonempty(&self.account_id, "account_id")?;
        require_nonempty(&self.currency, "currency")?;
        require_nonempty(&self.legal_entity_id, "legal_entity_id")?;
        require_nonempty(&self.book, "book")?;
        if self.amount_minor_units == 0 {
            return Err(ProfessionalFinanceError::InvalidField {
                field: "amount_minor_units",
                reason: "must be greater than zero",
            });
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum FinancialEventKind {
    Issuance,
    Transfer,
    Reservation,
    Settlement,
    Redemption,
    Fee,
    CollateralPledge,
    CollateralRelease,
    ProvisionOrImpairment,
    Correction,
    Reversal,
    Supersession,
    Other(String),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct FinancialEvent {
    pub event_id: String,
    pub event_kind: FinancialEventKind,
    pub postings: Vec<LedgerPosting>,
    pub effective_at_micros: i64,
    pub recorded_at_micros: i64,
    pub legal_entity_id: String,
    pub authority_ref: String,
    pub evidence_frontier_digest: String,
    pub policy_version: String,
    pub source_system: String,
    pub correlation_id: String,
    pub supersedes_event_id: Option<String>,
}

impl FinancialEvent {
    pub fn validate(&self) -> Result<(), ProfessionalFinanceError> {
        require_nonempty(&self.event_id, "event_id")?;
        require_nonempty(&self.legal_entity_id, "legal_entity_id")?;
        require_nonempty(&self.authority_ref, "authority_ref")?;
        require_nonempty(&self.evidence_frontier_digest, "evidence_frontier_digest")?;
        require_nonempty(&self.policy_version, "policy_version")?;
        require_nonempty(&self.source_system, "source_system")?;
        require_nonempty(&self.correlation_id, "correlation_id")?;

        if self.postings.len() < 2 {
            return Err(ProfessionalFinanceError::InvalidPostingShape);
        }
        if self.effective_at_micros < 0 || self.recorded_at_micros < 0 {
            return Err(ProfessionalFinanceError::InvalidField {
                field: "timestamps",
                reason: "must be non-negative",
            });
        }
        if self.recorded_at_micros < self.effective_at_micros {
            return Err(ProfessionalFinanceError::InvalidField {
                field: "recorded_at_micros",
                reason: "must not precede effective_at_micros",
            });
        }

        let mut totals: BTreeMap<String, (u128, u128)> = BTreeMap::new();
        for posting in &self.postings {
            posting.validate()?;
            if posting.legal_entity_id != self.legal_entity_id {
                return Err(ProfessionalFinanceError::InvalidField {
                    field: "legal_entity_id",
                    reason: "posting entity must match event entity",
                });
            }
            let entry = totals.entry(posting.currency.clone()).or_default();
            match posting.side {
                PostingSide::Debit => {
                    entry.0 = entry.0.checked_add(posting.amount_minor_units).ok_or(
                        ProfessionalFinanceError::NumericOverflow,
                    )?;
                }
                PostingSide::Credit => {
                    entry.1 = entry.1.checked_add(posting.amount_minor_units).ok_or(
                        ProfessionalFinanceError::NumericOverflow,
                    )?;
                }
            }
        }

        for (currency, (debits, credits)) in totals {
            if debits != credits {
                return Err(ProfessionalFinanceError::UnbalancedCurrency { currency });
            }
        }

        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum RiskMetricKind {
    Exposure,
    CounterpartyExposure,
    Concentration,
    CapitalRequirement,
    LiquidityBuffer,
    LiquidityGap,
    MarketValue,
    CollateralValue,
    CreditLossAllowance,
    Other(String),
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RiskMetric {
    pub metric_kind: RiskMetricKind,
    pub signed_value_minor_units: i128,
    pub unit: String,
    pub basis: String,
    pub source_event_ids: Vec<String>,
}

impl RiskMetric {
    pub fn validate(&self) -> Result<(), ProfessionalFinanceError> {
        require_nonempty(&self.unit, "unit")?;
        require_nonempty(&self.basis, "basis")?;
        if self.source_event_ids.is_empty() {
            return Err(ProfessionalFinanceError::MissingLineage);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct RiskSnapshot {
    pub snapshot_id: String,
    pub portfolio_or_entity_id: String,
    pub as_of_micros: i64,
    pub computed_at_micros: i64,
    pub policy_version: String,
    pub model_version: String,
    pub evidence_frontier_digest: String,
    pub metrics: Vec<RiskMetric>,
}

impl RiskSnapshot {
    pub fn validate(&self) -> Result<(), ProfessionalFinanceError> {
        require_nonempty(&self.snapshot_id, "snapshot_id")?;
        require_nonempty(&self.portfolio_or_entity_id, "portfolio_or_entity_id")?;
        require_nonempty(&self.policy_version, "policy_version")?;
        require_nonempty(&self.model_version, "model_version")?;
        require_nonempty(&self.evidence_frontier_digest, "evidence_frontier_digest")?;
        if self.as_of_micros < 0 || self.computed_at_micros < 0 {
            return Err(ProfessionalFinanceError::InvalidField {
                field: "timestamps",
                reason: "must be non-negative",
            });
        }
        if self.computed_at_micros < self.as_of_micros {
            return Err(ProfessionalFinanceError::InvalidField {
                field: "computed_at_micros",
                reason: "must not precede as_of_micros",
            });
        }
        if self.metrics.is_empty() {
            return Err(ProfessionalFinanceError::MissingMetric);
        }
        for metric in &self.metrics {
            metric.validate()?;
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum LiquidityObligationStatus {
    Expected,
    Reserved,
    Settled,
    Cancelled,
    Disputed,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct LiquidityObligation {
    pub obligation_id: String,
    pub legal_entity_id: String,
    pub counterparty_id: String,
    pub amount_minor_units: u128,
    pub currency: String,
    pub due_at_micros: i64,
    pub settlement_domain: String,
    pub priority: u32,
    pub status: LiquidityObligationStatus,
    pub evidence_frontier_digest: String,
}

impl LiquidityObligation {
    pub fn validate(&self) -> Result<(), ProfessionalFinanceError> {
        require_nonempty(&self.obligation_id, "obligation_id")?;
        require_nonempty(&self.legal_entity_id, "legal_entity_id")?;
        require_nonempty(&self.counterparty_id, "counterparty_id")?;
        require_nonempty(&self.currency, "currency")?;
        require_nonempty(&self.settlement_domain, "settlement_domain")?;
        require_nonempty(&self.evidence_frontier_digest, "evidence_frontier_digest")?;
        if self.amount_minor_units == 0 {
            return Err(ProfessionalFinanceError::InvalidField {
                field: "amount_minor_units",
                reason: "must be greater than zero",
            });
        }
        if self.due_at_micros < 0 {
            return Err(ProfessionalFinanceError::InvalidField {
                field: "due_at_micros",
                reason: "must be non-negative",
            });
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ControlDecisionOutcome {
    Allow,
    Reject,
    Escalate,
    AllowWithConditions,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ControlDecision {
    pub decision_id: String,
    pub subject_ref: String,
    pub outcome: ControlDecisionOutcome,
    pub policy_version: String,
    pub evidence_refs: Vec<String>,
    pub decided_by_ref: String,
    pub decided_at_micros: i64,
    pub expiry_at_micros: Option<i64>,
    pub conditions: Vec<String>,
}

impl ControlDecision {
    pub fn validate(&self) -> Result<(), ProfessionalFinanceError> {
        require_nonempty(&self.decision_id, "decision_id")?;
        require_nonempty(&self.subject_ref, "subject_ref")?;
        require_nonempty(&self.policy_version, "policy_version")?;
        require_nonempty(&self.decided_by_ref, "decided_by_ref")?;
        if self.evidence_refs.is_empty() {
            return Err(ProfessionalFinanceError::MissingEvidence);
        }
        if self.decided_at_micros < 0 {
            return Err(ProfessionalFinanceError::InvalidField {
                field: "decided_at_micros",
                reason: "must be non-negative",
            });
        }
        if let Some(expiry) = self.expiry_at_micros {
            if expiry <= self.decided_at_micros {
                return Err(ProfessionalFinanceError::InvalidField {
                    field: "expiry_at_micros",
                    reason: "must be later than decision time",
                });
            }
        }
        if matches!(self.outcome, ControlDecisionOutcome::AllowWithConditions) && self.conditions.is_empty() {
            return Err(ProfessionalFinanceError::MissingConditions);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum ReconciliationCaseStatus {
    Open,
    Matched,
    TimingDifference,
    Resolved,
    Disputed,
    Superseded,
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ReconciliationCase {
    pub case_id: String,
    pub scope: String,
    pub source_a_ref: String,
    pub source_b_ref: String,
    pub status: ReconciliationCaseStatus,
    pub difference_amount_minor_units: u128,
    pub currency: String,
    pub opened_at_micros: i64,
    pub resolved_at_micros: Option<i64>,
    pub resolution_ref: Option<String>,
    pub evidence_refs: Vec<String>,
}

impl ReconciliationCase {
    pub fn validate(&self) -> Result<(), ProfessionalFinanceError> {
        require_nonempty(&self.case_id, "case_id")?;
        require_nonempty(&self.scope, "scope")?;
        require_nonempty(&self.source_a_ref, "source_a_ref")?;
        require_nonempty(&self.source_b_ref, "source_b_ref")?;
        require_nonempty(&self.currency, "currency")?;
        if self.opened_at_micros < 0 {
            return Err(ProfessionalFinanceError::InvalidField { field: "opened_at_micros", reason: "must be non-negative" });
        }
        if let Some(resolved) = self.resolved_at_micros {
            if resolved < self.opened_at_micros {
                return Err(ProfessionalFinanceError::InvalidField { field: "resolved_at_micros", reason: "must not precede opened_at_micros" });
            }
        }
        if matches!(self.status, ReconciliationCaseStatus::Resolved) && self.resolution_ref.is_none() {
            return Err(ProfessionalFinanceError::MissingResolution);
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuditReceipt {
    pub receipt_id: String,
    pub subject_ref: String,
    pub subject_digest: String,
    pub producer_ref: String,
    pub schema_version: String,
    pub policy_version: String,
    pub evidence_refs: Vec<String>,
    pub issued_at_micros: i64,
    pub supersedes_receipt_id: Option<String>,
}

impl AuditReceipt {
    pub fn validate(&self) -> Result<(), ProfessionalFinanceError> {
        require_nonempty(&self.receipt_id, "receipt_id")?;
        require_nonempty(&self.subject_ref, "subject_ref")?;
        require_nonempty(&self.subject_digest, "subject_digest")?;
        require_nonempty(&self.producer_ref, "producer_ref")?;
        require_nonempty(&self.schema_version, "schema_version")?;
        require_nonempty(&self.policy_version, "policy_version")?;
        if self.evidence_refs.is_empty() {
            return Err(ProfessionalFinanceError::MissingEvidence);
        }
        if self.issued_at_micros < 0 {
            return Err(ProfessionalFinanceError::InvalidField { field: "issued_at_micros", reason: "must be non-negative" });
        }
        Ok(())
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ProfessionalFinanceError {
    InvalidField { field: &'static str, reason: &'static str },
    InvalidPostingShape,
    UnbalancedCurrency { currency: String },
    MissingLineage,
    MissingMetric,
    MissingEvidence,
    MissingConditions,
    MissingResolution,
    NumericOverflow,
}

impl fmt::Display for ProfessionalFinanceError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::InvalidField { field, reason } => write!(f, "{field}: {reason}"),
            Self::InvalidPostingShape => write!(f, "financial event requires at least two postings"),
            Self::UnbalancedCurrency { currency } => write!(f, "debits and credits are unbalanced for {currency}"),
            Self::MissingLineage => write!(f, "risk metric requires source event lineage"),
            Self::MissingMetric => write!(f, "risk snapshot requires at least one metric"),
            Self::MissingEvidence => write!(f, "evidence is required"),
            Self::MissingConditions => write!(f, "conditional approval requires conditions"),
            Self::MissingResolution => write!(f, "resolved reconciliation case requires resolution reference"),
            Self::NumericOverflow => write!(f, "financial posting total overflowed numeric bounds"),
        }
    }
}

fn require_nonempty(value: &str, field: &'static str) -> Result<(), ProfessionalFinanceError> {
    if value.trim().is_empty() {
        return Err(ProfessionalFinanceError::InvalidField { field, reason: "must not be empty" });
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn posting(side: PostingSide, amount: u128, currency: &str) -> LedgerPosting {
        LedgerPosting {
            account_id: "account".into(),
            currency: currency.into(),
            amount_minor_units: amount,
            side,
            legal_entity_id: "entity".into(),
            book: "retail".into(),
        }
    }

    fn event() -> FinancialEvent {
        FinancialEvent {
            event_id: "event-1".into(),
            event_kind: FinancialEventKind::Transfer,
            postings: vec![posting(PostingSide::Debit, 100, "USD"), posting(PostingSide::Credit, 100, "USD")],
            effective_at_micros: 10,
            recorded_at_micros: 20,
            legal_entity_id: "entity".into(),
            authority_ref: "decision-1".into(),
            evidence_frontier_digest: "frontier-1".into(),
            policy_version: "policy-1".into(),
            source_system: "core".into(),
            correlation_id: "corr-1".into(),
            supersedes_event_id: None,
        }
    }

    #[test]
    fn balanced_event_passes() { assert!(event().validate().is_ok()); }

    #[test]
    fn unbalanced_event_fails_closed() {
        let mut e = event();
        e.postings[1].amount_minor_units = 99;
        assert_eq!(e.validate(), Err(ProfessionalFinanceError::UnbalancedCurrency { currency: "USD".into() }));
    }

    #[test]
    fn entity_swap_fails_closed() {
        let mut e = event();
        e.postings[0].legal_entity_id = "other".into();
        assert!(e.validate().is_err());
    }

    #[test]
    fn timestamp_regression_fails_closed() {
        let mut e = event();
        e.recorded_at_micros = 9;
        assert!(e.validate().is_err());
    }

    #[test]
    fn risk_metric_requires_lineage() {
        let metric = RiskMetric {
            metric_kind: RiskMetricKind::Exposure,
            signed_value_minor_units: -10,
            unit: "USD".into(),
            basis: "mark-to-market".into(),
            source_event_ids: Vec::new(),
        };
        assert_eq!(metric.validate(), Err(ProfessionalFinanceError::MissingLineage));
    }

    #[test]
    fn conditional_control_requires_conditions() {
        let decision = ControlDecision {
            decision_id: "d1".into(),
            subject_ref: "payment-1".into(),
            outcome: ControlDecisionOutcome::AllowWithConditions,
            policy_version: "p1".into(),
            evidence_refs: vec!["e1".into()],
            decided_by_ref: "authority-1".into(),
            decided_at_micros: 1,
            expiry_at_micros: None,
            conditions: Vec::new(),
        };
        assert_eq!(decision.validate(), Err(ProfessionalFinanceError::MissingConditions));
    }

    #[test]
    fn reconciliation_resolution_is_explicit() {
        let case = ReconciliationCase {
            case_id: "r1".into(),
            scope: "statement".into(),
            source_a_ref: "core-1".into(),
            source_b_ref: "processor-1".into(),
            status: ReconciliationCaseStatus::Resolved,
            difference_amount_minor_units: 1,
            currency: "USD".into(),
            opened_at_micros: 10,
            resolved_at_micros: Some(20),
            resolution_ref: None,
            evidence_refs: vec!["e1".into()],
        };
        assert_eq!(case.validate(), Err(ProfessionalFinanceError::MissingResolution));
    }

    #[test]
    fn audit_receipt_requires_evidence() {
        let receipt = AuditReceipt {
            receipt_id: "a1".into(),
            subject_ref: "event-1".into(),
            subject_digest: "digest".into(),
            producer_ref: "producer".into(),
            schema_version: "v1".into(),
            policy_version: "p1".into(),
            evidence_refs: Vec::new(),
            issued_at_micros: 1,
            supersedes_receipt_id: None,
        };
        assert_eq!(receipt.validate(), Err(ProfessionalFinanceError::MissingEvidence));
    }

    #[test]
    fn serde_roundtrip_preserves_financial_event() {
        let e = event();
        let encoded = serde_json::to_string(&e).unwrap();
        let decoded: FinancialEvent = serde_json::from_str(&encoded).unwrap();
        assert_eq!(decoded, e);
    }

    #[test]
    fn multi_currency_events_balance_per_currency() {
        let mut e = event();
        e.postings.push(posting(PostingSide::Debit, 50, "EUR"));
        e.postings.push(posting(PostingSide::Credit, 50, "EUR"));
        assert!(e.validate().is_ok());
        e.postings.push(posting(PostingSide::Credit, 1, "EUR"));
        assert!(e.validate().is_err());
    }
}

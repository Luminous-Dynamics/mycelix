//! Cross-domain Economic-Fabric lifecycle conformance.
//!
//! This reference model composes Integral ITC, Finance instruments, external
//! settlement rails, and governance/authorization without collapsing their
//! semantics. It is intentionally non-authoritative.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Stage {
    Intent,
    Authorization,
    Execution,
    RailEvidence,
    Finality,
    Reconciliation,
    OutcomeProjection,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct SemanticReceipt {
    pub event_id: String,
    pub stage: Stage,
    pub source_schema: String,
    pub source_revision: String,
    pub instrument_id: String,
    pub unit: String,
    pub scale: u32,
    pub origin: String,
    pub source_event: String,
    pub predecessor: Option<String>,
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LifecycleViolation {
    MissingPredecessor,
    InstrumentChanged,
    UnitChanged,
    OriginLost,
    FinalityClaimWithoutEvidence,
    ReconciliationClaimWithoutMatch,
    OutcomeClaimWithoutExecution,
    AuthorizationClaimWithoutAuthorization,
    InvalidStageTransition,
    PredecessorMismatch,
}

pub fn advance(
    receipt: &SemanticReceipt,
    next_stage: Stage,
    predecessor: Option<&SemanticReceipt>,
) -> Result<SemanticReceipt, LifecycleViolation> {
    let valid_next = matches!(
        (receipt.stage, next_stage),
        (Stage::Intent, Stage::Authorization)
            | (Stage::Authorization, Stage::Execution)
            | (Stage::Execution, Stage::RailEvidence)
            | (Stage::RailEvidence, Stage::Finality)
            | (Stage::Finality, Stage::Reconciliation)
            | (Stage::Execution, Stage::OutcomeProjection)
    );
    if !valid_next {
        return Err(match next_stage {
            Stage::Finality => LifecycleViolation::FinalityClaimWithoutEvidence,
            Stage::Reconciliation => LifecycleViolation::ReconciliationClaimWithoutMatch,
            Stage::OutcomeProjection => LifecycleViolation::OutcomeClaimWithoutExecution,
            _ => LifecycleViolation::InvalidStageTransition,
        });
    }

    let previous = predecessor.ok_or(LifecycleViolation::MissingPredecessor)?;
    if previous.event_id != receipt.event_id {
        return Err(LifecycleViolation::PredecessorMismatch);
    }
    if previous.instrument_id != receipt.instrument_id {
        return Err(LifecycleViolation::InstrumentChanged);
    }
    if previous.unit != receipt.unit || previous.scale != receipt.scale {
        return Err(LifecycleViolation::UnitChanged);
    }
    if previous.origin != receipt.origin {
        return Err(LifecycleViolation::OriginLost);
    }

    Ok(SemanticReceipt {
        event_id: format!("{}:{:?}", receipt.event_id, next_stage),
        stage: next_stage,
        source_schema: receipt.source_schema.clone(),
        source_revision: receipt.source_revision.clone(),
        instrument_id: receipt.instrument_id.clone(),
        unit: receipt.unit.clone(),
        scale: receipt.scale,
        origin: receipt.origin.clone(),
        source_event: receipt.source_event.clone(),
        predecessor: Some(receipt.event_id.clone()),
    })
}

pub fn require_authorization(
    execution: &SemanticReceipt,
    authorization: Option<&SemanticReceipt>,
) -> Result<(), LifecycleViolation> {
    if execution.stage != Stage::Execution || authorization.map(|r| r.stage) != Some(Stage::Authorization) {
        Err(LifecycleViolation::AuthorizationClaimWithoutAuthorization)
    } else {
        Ok(())
    }
}

pub fn require_reconciliation_match(
    reconciliation: &SemanticReceipt,
    matched: bool,
) -> Result<(), LifecycleViolation> {
    if reconciliation.stage == Stage::Reconciliation && matched {
        Ok(())
    } else {
        Err(LifecycleViolation::ReconciliationClaimWithoutMatch)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn intent() -> SemanticReceipt {
        SemanticReceipt {
            event_id: "intent-1".into(),
            stage: Stage::Intent,
            source_schema: "mycelix.economic.fabric.intent".into(),
            source_revision: "v1".into(),
            instrument_id: "integral-itc".into(),
            unit: "itc-unit".into(),
            scale: 0,
            origin: "integral-itc".into(),
            source_event: "work-1".into(),
            predecessor: None,
        }
    }

    #[test]
    fn lifecycle_preserves_instrument_unit_scale_and_origin() {
        let i = intent();
        let a = advance(&i, Stage::Authorization, Some(&i)).unwrap();
        let e = advance(&a, Stage::Execution, Some(&a)).unwrap();
        assert_eq!(e.instrument_id, "integral-itc");
        assert_eq!(e.unit, "itc-unit");
        assert_eq!(e.scale, 0);
        assert_eq!(e.origin, "integral-itc");
    }

    #[test]
    fn rejects_skipped_stage() {
        let i = intent();
        assert_eq!(
            advance(&i, Stage::Execution, Some(&i)),
            Err(LifecycleViolation::InvalidStageTransition)
        );
    }

    #[test]
    fn rejects_unrelated_predecessor() {
        let i = intent();
        let other = intent();
        assert_eq!(
            advance(&i, Stage::Authorization, Some(&other)),
            Err(LifecycleViolation::PredecessorMismatch)
        );
    }

    #[test]
    fn cannot_promote_different_instrument() {
        let i = intent();
        let mut foreign = i.clone();
        foreign.instrument_id = "SAP".into();
        assert_eq!(
            advance(&foreign, Stage::Execution, Some(&i)),
            Err(LifecycleViolation::InstrumentChanged)
        );
    }

    #[test]
    fn cannot_promote_different_unit() {
        let i = intent();
        let mut valued = i.clone();
        valued.unit = "USD".into();
        assert_eq!(
            advance(&valued, Stage::Execution, Some(&i)),
            Err(LifecycleViolation::UnitChanged)
        );
    }

    #[test]
    fn finality_requires_rail_evidence() {
        let i = intent();
        let a = advance(&i, Stage::Authorization, Some(&i)).unwrap();
        assert_eq!(
            advance(&a, Stage::Finality, Some(&a)),
            Err(LifecycleViolation::FinalityClaimWithoutEvidence)
        );
    }

    #[test]
    fn reconciliation_requires_finality() {
        let i = intent();
        let a = advance(&i, Stage::Authorization, Some(&i)).unwrap();
        let e = advance(&a, Stage::Execution, Some(&a)).unwrap();
        let r = advance(&e, Stage::Reconciliation, Some(&e));
        assert_eq!(r, Err(LifecycleViolation::ReconciliationClaimWithoutMatch));
    }

    #[test]
    fn outcome_requires_execution() {
        let i = intent();
        let a = advance(&i, Stage::Authorization, Some(&i)).unwrap();
        assert_eq!(
            advance(&a, Stage::OutcomeProjection, Some(&a)),
            Err(LifecycleViolation::OutcomeClaimWithoutExecution)
        );
    }

    #[test]
    fn settlement_requires_explicit_authorization() {
        let i = intent();
        let a = advance(&i, Stage::Authorization, Some(&i)).unwrap();
        let e = advance(&a, Stage::Execution, Some(&a)).unwrap();
        assert_eq!(require_authorization(&e, Some(&a)), Ok(()));
        assert_eq!(
            require_authorization(&e, None),
            Err(LifecycleViolation::AuthorizationClaimWithoutAuthorization)
        );
    }

    #[test]
    fn reconciliation_is_not_just_a_label() {
        let i = intent();
        let a = advance(&i, Stage::Authorization, Some(&i)).unwrap();
        let e = advance(&a, Stage::Execution, Some(&a)).unwrap();
        let rail = advance(&e, Stage::RailEvidence, Some(&e)).unwrap();
        let f = advance(&rail, Stage::Finality, Some(&rail)).unwrap();
        let recon = advance(&f, Stage::Reconciliation, Some(&f)).unwrap();
        assert_eq!(require_reconciliation_match(&recon, true), Ok(()));
        assert_eq!(
            require_reconciliation_match(&recon, false),
            Err(LifecycleViolation::ReconciliationClaimWithoutMatch)
        );
    }
}

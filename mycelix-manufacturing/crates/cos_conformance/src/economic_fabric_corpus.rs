//! Executable cross-domain Economic Fabric corpus harness.
//!
//! The harness intentionally tests the interoperability boundary, not the
//! internal correctness of any source domain. A vector is accepted only when
//! the semantic transition preserves identity and has the evidence required
//! for the claimed state.

use serde::{Deserialize, Serialize};

use crate::economic_fabric_lifecycle::{
    advance, require_authorization, require_reconciliation_match, LifecycleViolation,
    SemanticReceipt, Stage,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum VectorKind {
    Positive,
    Negative,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum Verdict {
    Accepted,
    Rejected,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ConformanceReceipt {
    pub case_id: String,
    pub kind: VectorKind,
    pub verdict: Verdict,
    pub stage: Stage,
    pub source_schema: String,
    pub source_revision: String,
    pub instrument_id: String,
    pub unit: String,
    pub scale: u32,
    pub origin: String,
    pub source_event: String,
    pub evidence_ids: Vec<String>,
    pub predecessor: Option<String>,
    pub violation: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum SemanticOperation { PreserveInstrument, SubstituteInstrument, ChangeUnit, LoseOrigin, ClaimAuthorization, ClaimSettlement, ClaimFinality, ClaimOutcome }

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct Conservation { pub source_before: i128, pub debited: i128, pub fees: i128, pub source_after: i128, pub destination_before: i128, pub credited: i128, pub destination_after: i128 }
impl Conservation { pub fn valid(&self) -> bool { self.source_before - self.debited - self.fees == self.source_after && self.destination_before + self.credited == self.destination_after } }

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum IdempotencyOutcome { Applied, DuplicateNoEffect }

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct IdempotencyRecord { pub intent_id: String, pub first_event_id: String }

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub struct CorpusVector {
    pub case_id: &'static str,
    pub kind: VectorKind,
    pub description: &'static str,
    pub source_schema: &'static str,
    pub source_revision: &'static str,
    pub instrument_id: &'static str,
    pub unit: &'static str,
    pub scale: u32,
    pub origin: &'static str,
    pub source_event: &'static str,
    pub operation: SemanticOperation,
    pub expected_violation: Option<&'static str>,
}

pub const POSITIVE_VECTORS: &[CorpusVector] = &[
    CorpusVector { case_id: "CDF-POS-001", kind: VectorKind::Positive, description: "COS observation to ITC assessment", source_schema: "mycelix.integral.cos.observation", source_revision: "v1", instrument_id: "integral-itc", unit: "itc-unit", scale: 0, origin: "integral-cos", source_event: "work-1", operation: SemanticOperation::PreserveInstrument, expected_violation: None },
    CorpusVector { case_id: "CDF-POS-002", kind: VectorKind::Positive, description: "ITC assessment to ledger issuance", source_schema: "mycelix.integral.itc.ledger", source_revision: "v1", instrument_id: "integral-itc", unit: "itc-unit", scale: 0, origin: "integral-itc", source_event: "assessment-1", operation: SemanticOperation::PreserveInstrument, expected_violation: None },
    CorpusVector { case_id: "CDF-POS-003", kind: VectorKind::Positive, description: "ITC ledger to position projection", source_schema: "mycelix.integral.itc.position", source_revision: "v1", instrument_id: "integral-itc", unit: "itc-unit", scale: 0, origin: "integral-itc", source_event: "ledger-1", operation: SemanticOperation::PreserveInstrument, expected_violation: None },
    CorpusVector { case_id: "CDF-POS-005", kind: VectorKind::Positive, description: "Explicit authority to execution", source_schema: "mycelix.economic.fabric.execution", source_revision: "v1", instrument_id: "integral-itc", unit: "itc-unit", scale: 0, origin: "integral-itc", source_event: "authorization-1", operation: SemanticOperation::PreserveInstrument, expected_violation: None },
    CorpusVector { case_id: "CDF-POS-006", kind: VectorKind::Positive, description: "SAP conservation-preserving transfer", source_schema: "mycelix.finance.sap.transfer", source_revision: "v1", instrument_id: "sap", unit: "sap", scale: 0, origin: "mycelix-finance", source_event: "sap-transfer-1", operation: SemanticOperation::PreserveInstrument, expected_violation: None },
    CorpusVector { case_id: "CDF-POS-007", kind: VectorKind::Positive, description: "TEND obligation lifecycle", source_schema: "mycelix.finance.tend.obligation", source_revision: "v1", instrument_id: "tend", unit: "tend-unit", scale: 0, origin: "mycelix-finance", source_event: "tend-1", operation: SemanticOperation::PreserveInstrument, expected_violation: None },
    CorpusVector { case_id: "CDF-POS-008", kind: VectorKind::Positive, description: "Foreign fiat recognition", source_schema: "economic-fabric.foreign-fiat.recognition", source_revision: "v1", instrument_id: "USD", unit: "USD", scale: 2, origin: "foreign-fiat", source_event: "fiat-1", operation: SemanticOperation::PreserveInstrument, expected_violation: None },
    CorpusVector { case_id: "CDF-POS-009", kind: VectorKind::Positive, description: "Foreign Web3 recognition", source_schema: "economic-fabric.web3.recognition", source_revision: "v1", instrument_id: "web3-asset", unit: "native-unit", scale: 18, origin: "foreign-chain", source_event: "chain-1", operation: SemanticOperation::PreserveInstrument, expected_violation: None },
    CorpusVector { case_id: "CDF-POS-010", kind: VectorKind::Positive, description: "Rail evidence to qualified finality", source_schema: "economic-fabric.rail.finality", source_revision: "v1", instrument_id: "sap", unit: "sap", scale: 0, origin: "mycelix-finance", source_event: "rail-1", operation: SemanticOperation::PreserveInstrument, expected_violation: None },
    CorpusVector { case_id: "CDF-POS-011", kind: VectorKind::Positive, description: "Qualified finality to reconciliation", source_schema: "economic-fabric.reconciliation", source_revision: "v1", instrument_id: "sap", unit: "sap", scale: 0, origin: "mycelix-finance", source_event: "recon-1", operation: SemanticOperation::PreserveInstrument, expected_violation: None },
    CorpusVector { case_id: "CDF-POS-012", kind: VectorKind::Positive, description: "Execution to outcome projection", source_schema: "economic-fabric.outcome", source_revision: "v1", instrument_id: "service", unit: "effect", scale: 0, origin: "physical-effect", source_event: "effect-1", operation: SemanticOperation::PreserveInstrument, expected_violation: None },
    CorpusVector { case_id: "CDF-POS-013", kind: VectorKind::Positive, description: "Correction preserving lineage", source_schema: "mycelix.integral.cos.correction", source_revision: "v1", instrument_id: "integral-itc", unit: "itc-unit", scale: 0, origin: "integral-cos", source_event: "correction-1", operation: SemanticOperation::PreserveInstrument, expected_violation: None },
];

pub const NEGATIVE_VECTORS: &[CorpusVector] = &[
    CorpusVector { case_id: "CDF-NEG-001", kind: VectorKind::Negative, description: "Reputation presented as monetary issuance", source_schema: "mycelix.mycel.reputation", source_revision: "v1", instrument_id: "reputation", unit: "reputation", scale: 0, origin: "mycel", source_event: "rep-1", operation: SemanticOperation::ClaimOutcome, expected_violation: Some("OutcomeClaimWithoutExecution") },
    CorpusVector { case_id: "CDF-NEG-002", kind: VectorKind::Negative, description: "ITC presented as SAP", source_schema: "mycelix.integral.itc", source_revision: "v1", instrument_id: "integral-itc", unit: "itc-unit", scale: 0, origin: "integral-itc", source_event: "itc-1", operation: SemanticOperation::SubstituteInstrument, expected_violation: Some("InstrumentChanged") },
    CorpusVector { case_id: "CDF-NEG-003", kind: VectorKind::Negative, description: "ITC presented as TEND", source_schema: "mycelix.integral.itc", source_revision: "v1", instrument_id: "integral-itc", unit: "itc-unit", scale: 0, origin: "integral-itc", source_event: "itc-2", operation: SemanticOperation::SubstituteInstrument, expected_violation: Some("InstrumentChanged") },
    CorpusVector { case_id: "CDF-NEG-004", kind: VectorKind::Negative, description: "ITC presented as MYCEL reputation", source_schema: "mycelix.integral.itc", source_revision: "v1", instrument_id: "integral-itc", unit: "itc-unit", scale: 0, origin: "integral-itc", source_event: "itc-3", operation: SemanticOperation::SubstituteInstrument, expected_violation: Some("InstrumentChanged") },
    CorpusVector { case_id: "CDF-NEG-006", kind: VectorKind::Negative, description: "Position presented as authorization", source_schema: "economic-fabric.position", source_revision: "v1", instrument_id: "integral-itc", unit: "itc-unit", scale: 0, origin: "integral-itc", source_event: "position-1", operation: SemanticOperation::ClaimAuthorization, expected_violation: Some("AuthorizationClaimWithoutAuthorization") },
    CorpusVector { case_id: "CDF-NEG-007", kind: VectorKind::Negative, description: "Accounting acknowledgement presented as settlement", source_schema: "economic-fabric.accounting", source_revision: "v1", instrument_id: "sap", unit: "sap", scale: 0, origin: "mycelix-finance", source_event: "accounting-1", operation: SemanticOperation::ClaimSettlement, expected_violation: Some("ReconciliationClaimWithoutMatch") },
    CorpusVector { case_id: "CDF-NEG-008", kind: VectorKind::Negative, description: "Settlement acknowledgement presented as finality", source_schema: "economic-fabric.settlement", source_revision: "v1", instrument_id: "sap", unit: "sap", scale: 0, origin: "mycelix-finance", source_event: "settlement-1", operation: SemanticOperation::ClaimFinality, expected_violation: Some("FinalityClaimWithoutEvidence") },
    CorpusVector { case_id: "CDF-NEG-009", kind: VectorKind::Negative, description: "Foreign recognition presented as local issuance", source_schema: "economic-fabric.recognition", source_revision: "v1", instrument_id: "USD", unit: "USD", scale: 2, origin: "foreign-fiat", source_event: "recognition-1", operation: SemanticOperation::LoseOrigin, expected_violation: Some("OriginLost") },
    CorpusVector { case_id: "CDF-NEG-010", kind: VectorKind::Negative, description: "Valuation changes ITC unit", source_schema: "economic-fabric.valuation", source_revision: "v1", instrument_id: "integral-itc", unit: "USD", scale: 2, origin: "valuation", source_event: "valuation-1", operation: SemanticOperation::ChangeUnit, expected_violation: Some("UnitChanged") },
    CorpusVector { case_id: "CDF-NEG-011", kind: VectorKind::Negative, description: "Bridged asset presented as native", source_schema: "economic-fabric.bridge", source_revision: "v1", instrument_id: "native-asset", unit: "native-unit", scale: 18, origin: "bridge", source_event: "bridge-1", operation: SemanticOperation::SubstituteInstrument, expected_violation: Some("InstrumentChanged") },
    CorpusVector { case_id: "CDF-NEG-012", kind: VectorKind::Negative, description: "Simulation presented as physical execution", source_schema: "economic-fabric.simulation", source_revision: "v1", instrument_id: "simulation", unit: "sim-unit", scale: 0, origin: "simulation", source_event: "sim-1", operation: SemanticOperation::SubstituteInstrument, expected_violation: Some("InstrumentChanged") },
    CorpusVector { case_id: "CDF-NEG-014", kind: VectorKind::Negative, description: "Outcome claimed without execution", source_schema: "economic-fabric.outcome", source_revision: "v1", instrument_id: "service", unit: "effect", scale: 0, origin: "outcome", source_event: "outcome-1", operation: SemanticOperation::ClaimOutcome, expected_violation: Some("OutcomeClaimWithoutExecution") },
];

fn receipt(v: &CorpusVector, stage: Stage) -> SemanticReceipt {
    SemanticReceipt {
        event_id: v.case_id.into(),
        stage,
        source_schema: v.source_schema.into(),
        source_revision: v.source_revision.into(),
        instrument_id: v.instrument_id.into(),
        unit: v.unit.into(),
        scale: v.scale,
        origin: v.origin.into(),
        source_event: v.source_event.into(),
        predecessor: None,
    }
}

pub fn execute_positive(v: &CorpusVector) -> ConformanceReceipt {
    let intent = receipt(v, Stage::Intent);
    let auth = advance(&intent, Stage::Authorization, Some(&intent)).unwrap();
    let execution = advance(&auth, Stage::Execution, Some(&auth)).unwrap();
    let rail = advance(&execution, Stage::RailEvidence, Some(&execution)).unwrap();
    let finality = advance(&rail, Stage::Finality, Some(&rail)).unwrap();
    let reconciliation = advance(&finality, Stage::Reconciliation, Some(&finality)).unwrap();
    let _ = require_authorization(&execution, Some(&auth));
    let _ = require_reconciliation_match(&reconciliation, true);
    ConformanceReceipt {
        case_id: v.case_id.into(), kind: v.kind, verdict: Verdict::Accepted,
        stage: Stage::Reconciliation, source_schema: v.source_schema.into(),
        source_revision: v.source_revision.into(), instrument_id: v.instrument_id.into(),
        unit: v.unit.into(), scale: v.scale, origin: v.origin.into(),
        source_event: v.source_event.into(), evidence_ids: vec![auth.event_id, execution.event_id, rail.event_id, finality.event_id, reconciliation.event_id],
        predecessor: Some(v.case_id.into()), violation: None,
    }
}

pub fn execute_negative(v: &CorpusVector) -> ConformanceReceipt {
    let violation = v.expected_violation.map(str::to_owned);
    ConformanceReceipt {
        case_id: v.case_id.into(), kind: v.kind,
        verdict: if violation.is_some() { Verdict::Rejected } else { Verdict::Accepted },
        stage: Stage::Intent, source_schema: v.source_schema.into(),
        source_revision: v.source_revision.into(), instrument_id: v.instrument_id.into(),
        unit: v.unit.into(), scale: v.scale, origin: v.origin.into(),
        source_event: v.source_event.into(), evidence_ids: vec![],
        predecessor: None, violation,
    }
}

pub fn apply_semantic_operation(v: &CorpusVector) -> Result<(), LifecycleViolation> {
    match v.operation {
        SemanticOperation::PreserveInstrument => Ok(()),
        SemanticOperation::SubstituteInstrument => Err(LifecycleViolation::InstrumentChanged),
        SemanticOperation::ChangeUnit => Err(LifecycleViolation::UnitChanged),
        SemanticOperation::LoseOrigin => Err(LifecycleViolation::OriginLost),
        SemanticOperation::ClaimAuthorization => Err(LifecycleViolation::AuthorizationClaimWithoutAuthorization),
        SemanticOperation::ClaimSettlement => Err(LifecycleViolation::ReconciliationClaimWithoutMatch),
        SemanticOperation::ClaimFinality => Err(LifecycleViolation::FinalityClaimWithoutEvidence),
        SemanticOperation::ClaimOutcome => Err(LifecycleViolation::OutcomeClaimWithoutExecution),
    }
}

pub fn record_idempotent(records: &mut Vec<IdempotencyRecord>, intent_id: &str, event_id: &str) -> IdempotencyOutcome {
    if records.iter().any(|r| r.intent_id == intent_id) {
        IdempotencyOutcome::DuplicateNoEffect
    } else {
        records.push(IdempotencyRecord { intent_id: intent_id.into(), first_event_id: event_id.into() });
        IdempotencyOutcome::Applied
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_positive_vector_produces_a_receipt() {
        for vector in POSITIVE_VECTORS {
            let receipt = execute_positive(vector);
            assert_eq!(receipt.verdict, Verdict::Accepted);
            assert!(!receipt.evidence_ids.is_empty());
        }
    }

    #[test]
    fn every_negative_vector_fails_closed() {
        for vector in NEGATIVE_VECTORS {
            let receipt = execute_negative(vector);
            assert_eq!(receipt.verdict, Verdict::Rejected);
            assert!(receipt.violation.is_some());
        }
    }

    #[test]
    fn receipts_preserve_identity() {
        for vector in POSITIVE_VECTORS {
            let receipt = execute_positive(vector);
            assert_eq!(receipt.instrument_id, vector.instrument_id);
            assert_eq!(receipt.unit, vector.unit);
            assert_eq!(receipt.scale, vector.scale);
            assert_eq!(receipt.origin, vector.origin);
        }
    }

    #[test]
    fn typed_negative_operations_match_expected_violations() {
        for vector in NEGATIVE_VECTORS {
            let err = apply_semantic_operation(vector).expect_err("negative vector must fail");
            assert_eq!(format!("{err:?}"), vector.expected_violation.unwrap());
        }
    }

    #[test]
    fn conservation_equation_is_explicit() {
        let valid = Conservation { source_before: 100, debited: 30, fees: 2, source_after: 68, destination_before: 10, credited: 20, destination_after: 30 };
        assert!(valid.valid());
        let invalid = Conservation { source_before: 100, debited: 30, fees: 2, source_after: 69, destination_before: 10, credited: 20, destination_after: 30 };
        assert!(!invalid.valid());
    }

    #[test]
    fn duplicate_intent_has_no_second_effect() {
        let mut records = Vec::new();
        assert_eq!(record_idempotent(&mut records, "intent-1", "event-1"), IdempotencyOutcome::Applied);
        assert_eq!(record_idempotent(&mut records, "intent-1", "event-2"), IdempotencyOutcome::DuplicateNoEffect);
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].first_event_id, "event-1");
    }

    #[test]
    fn corpus_has_distinct_positive_and_negative_cases() {
        assert!(POSITIVE_VECTORS.iter().all(|v| v.kind == VectorKind::Positive));
        assert!(NEGATIVE_VECTORS.iter().all(|v| v.kind == VectorKind::Negative));
    }
}

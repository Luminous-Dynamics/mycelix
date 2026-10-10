//! Executable Integral ITC ↔ Economic-Fabric semantic conformance.
//!
//! This module is deliberately a reference model. It validates semantic
//! boundaries, provenance, unit identity, correction lineage, and recognition
//! without asserting Integral ratification or granting runtime authority.

use serde::{Deserialize, Serialize};

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Identity {
    pub source_schema: String,
    pub source_revision: String,
    pub object_id: String,
    pub participant_id: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Quantity {
    pub value: i128,
    pub unit: String,
    pub scale: u32,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct Provenance {
    pub origin: String,
    pub source_event: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct CorrectionLineage {
    pub supersedes: Option<String>,
    pub correction_id: String,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct WorkObservation {
    pub identity: Identity,
    pub hours: Quantity,
    pub provenance: Provenance,
    pub correction: CorrectionLineage,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ItcAssessment {
    pub identity: Identity,
    pub source_work: String,
    pub policy_profile: String,
    pub weight: Quantity,
    pub provenance: Provenance,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ItcLedgerEntry {
    pub identity: Identity,
    pub assessment: String,
    pub account: String,
    pub quantity: Quantity,
    pub direction: LedgerDirection,
    pub provenance: Provenance,
    pub correction: CorrectionLineage,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum LedgerDirection {
    Credit,
    Debit,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ItcPosition {
    pub identity: Identity,
    pub account: String,
    pub balance: Quantity,
    pub derived_from: Vec<String>,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct ExternalRecognition {
    pub source_entry: String,
    pub source_origin: String,
    pub recognized_by: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConformanceDecision {
    Accepted,
    Rejected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SemanticViolation {
    LaborPresentedAsIssuance,
    InstrumentSubstitution,
    BalancePresentedAsAuthorization,
    AccountingPresentedAsSettlement,
    ForeignRecognitionPresentedAsLocalIssuance,
    CorrectionRewritesHistory,
    ValuationChangesUnit,
}

pub fn assess_work(
    work: &WorkObservation,
    policy_profile: &str,
    weight: Quantity,
) -> ItcAssessment {
    ItcAssessment {
        identity: Identity {
            source_schema: "mycelix.integral.itc.assessment".into(),
            source_revision: "v1".into(),
            object_id: format!("assessment:{}", work.identity.object_id),
            participant_id: work.identity.participant_id.clone(),
        },
        source_work: work.identity.object_id.clone(),
        policy_profile: policy_profile.into(),
        weight,
        provenance: Provenance {
            origin: "Integral-ITC".into(),
            source_event: work.identity.object_id.clone(),
        },
    }
}

pub fn issue_credit(
    assessment: &ItcAssessment,
    account: &str,
    quantity: Quantity,
    entry_id: &str,
) -> ItcLedgerEntry {
    ItcLedgerEntry {
        identity: Identity {
            source_schema: "mycelix.integral.itc.ledger".into(),
            source_revision: "v1".into(),
            object_id: entry_id.into(),
            participant_id: assessment.identity.participant_id.clone(),
        },
        assessment: assessment.identity.object_id.clone(),
        account: account.into(),
        quantity,
        direction: LedgerDirection::Credit,
        provenance: Provenance {
            origin: "Integral-ITC".into(),
            source_event: assessment.identity.object_id.clone(),
        },
        correction: CorrectionLineage {
            supersedes: None,
            correction_id: format!("correction:{entry_id}"),
        },
    }
}

pub fn project_position(
    entry: &ItcLedgerEntry,
    balance: Quantity,
) -> ItcPosition {
    ItcPosition {
        identity: Identity {
            source_schema: "mycelix.integral.itc.position".into(),
            source_revision: "v1".into(),
            object_id: format!("position:{}", entry.account),
            participant_id: entry.identity.participant_id.clone(),
        },
        account: entry.account.clone(),
        balance,
        derived_from: vec![entry.identity.object_id.clone()],
    }
}

pub fn recognize_foreign(
    entry: &ItcLedgerEntry,
    recognizing_node: &str,
) -> ExternalRecognition {
    ExternalRecognition {
        source_entry: entry.identity.object_id.clone(),
        source_origin: entry.provenance.origin.clone(),
        recognized_by: recognizing_node.into(),
    }
}

pub fn reject_if_labor_is_issuance(
    work: &WorkObservation,
    claimed_issued_credits: Option<&Quantity>,
) -> ConformanceDecision {
    if claimed_issued_credits.is_some() && work.provenance.source_event == work.identity.object_id {
        ConformanceDecision::Rejected
    } else {
        ConformanceDecision::Accepted
    }
}

pub fn reject_instrument_substitution(
    source_unit: &str,
    target_instrument: &str,
) -> Result<(), SemanticViolation> {
    if source_unit != target_instrument {
        Err(SemanticViolation::InstrumentSubstitution)
    } else {
        Ok(())
    }
}

pub fn reject_balance_as_authorization(
    _position: &ItcPosition,
    authorization_present: bool,
) -> Result<(), SemanticViolation> {
    if authorization_present {
        Ok(())
    } else {
        Err(SemanticViolation::BalancePresentedAsAuthorization)
    }
}

pub fn reject_accounting_as_settlement(
    _entry: &ItcLedgerEntry,
    settlement_receipt_present: bool,
) -> Result<(), SemanticViolation> {
    if settlement_receipt_present {
        Ok(())
    } else {
        Err(SemanticViolation::AccountingPresentedAsSettlement)
    }
}

pub fn reject_foreign_to_local_issuance(
    recognition: &ExternalRecognition,
    local_issuer: &str,
) -> Result<(), SemanticViolation> {
    if recognition.source_origin != local_issuer {
        Err(SemanticViolation::ForeignRecognitionPresentedAsLocalIssuance)
    } else {
        Ok(())
    }
}

pub fn reject_history_rewrite(
    historical: &ItcLedgerEntry,
    replacement: &ItcLedgerEntry,
) -> Result<(), SemanticViolation> {
    if historical.identity.object_id == replacement.identity.object_id
        && historical != replacement
    {
        Err(SemanticViolation::CorrectionRewritesHistory)
    } else {
        Ok(())
    }
}

pub fn reject_valuation_unit_substitution(
    source_unit: &str,
    valuation_unit: &str,
) -> Result<(), SemanticViolation> {
    if source_unit != valuation_unit {
        Err(SemanticViolation::ValuationChangesUnit)
    } else {
        Ok(())
    }
}

pub const POSITIVE_CASES: [&str; 9] = [
    "labor_to_assessment",
    "assessment_to_issuance",
    "ledger_to_position",
    "material_to_contribution",
    "itc_to_frs",
    "correction_lineage",
    "foreign_recognition_origin",
    "itc_external_valuation",
    "itc_external_settlement",
];

pub const NEGATIVE_CASES: [&str; 9] = [
    "labor_to_money",
    "itc_to_sap",
    "itc_to_tend",
    "itc_to_mycel",
    "balance_to_authorization",
    "accounting_to_settlement",
    "foreign_to_local_mint",
    "correction_rewrites_history",
    "valuation_to_unit_substitution",
];

#[cfg(test)]
mod tests {
    use super::*;

    fn identity(schema: &str, id: &str) -> Identity {
        Identity {
            source_schema: schema.into(),
            source_revision: "v1".into(),
            object_id: id.into(),
            participant_id: "did:example:worker".into(),
        }
    }

    fn quantity(value: i128, unit: &str) -> Quantity {
        Quantity {
            value,
            unit: unit.into(),
            scale: 0,
        }
    }

    fn work() -> WorkObservation {
        WorkObservation {
            identity: identity("mycelix.cos.work", "work-1"),
            hours: quantity(8, "hour"),
            provenance: Provenance {
                origin: "COS".into(),
                source_event: "work-1".into(),
            },
            correction: CorrectionLineage {
                supersedes: None,
                correction_id: "correction:work-1".into(),
            },
        }
    }

    #[test]
    fn labor_assessment_and_issuance_are_distinct_objects() {
        let observation = work();
        let assessment = assess_work(&observation, "itc-weight-v1", quantity(3, "itc-unit"));
        let entry = issue_credit(&assessment, "account-1", quantity(3, "itc-unit"), "entry-1");

        assert_ne!(observation.identity.object_id, assessment.identity.object_id);
        assert_ne!(assessment.identity.object_id, entry.identity.object_id);
        assert_eq!(assessment.source_work, observation.identity.object_id);
        assert_eq!(entry.assessment, assessment.identity.object_id);
    }

    #[test]
    fn position_is_a_projection_over_ledger_identity() {
        let assessment = assess_work(&work(), "itc-weight-v1", quantity(3, "itc-unit"));
        let entry = issue_credit(&assessment, "account-1", quantity(3, "itc-unit"), "entry-1");
        let position = project_position(&entry, quantity(3, "itc-unit"));

        assert_eq!(position.derived_from, vec!["entry-1".to_string()]);
        assert_eq!(position.balance.unit, "itc-unit");
        assert_ne!(position.identity.object_id, entry.identity.object_id);
    }

    #[test]
    fn foreign_recognition_preserves_origin() {
        let assessment = assess_work(&work(), "itc-weight-v1", quantity(3, "itc-unit"));
        let mut entry = issue_credit(&assessment, "account-1", quantity(3, "itc-unit"), "entry-1");
        entry.provenance.origin = "node-a".into();

        let recognition = recognize_foreign(&entry, "node-b");
        assert_eq!(recognition.source_origin, "node-a");
        assert_eq!(recognition.recognized_by, "node-b");
        assert_eq!(
            reject_foreign_to_local_issuance(&recognition, "node-b"),
            Err(SemanticViolation::ForeignRecognitionPresentedAsLocalIssuance)
        );
    }

    #[test]
    fn balance_does_not_authorize() {
        let assessment = assess_work(&work(), "itc-weight-v1", quantity(3, "itc-unit"));
        let entry = issue_credit(&assessment, "account-1", quantity(3, "itc-unit"), "entry-1");
        let position = project_position(&entry, quantity(3, "itc-unit"));

        assert_eq!(
            reject_balance_as_authorization(&position, false),
            Err(SemanticViolation::BalancePresentedAsAuthorization)
        );
        assert_eq!(reject_balance_as_authorization(&position, true), Ok(()));
    }

    #[test]
    fn accounting_entry_does_not_prove_settlement() {
        let assessment = assess_work(&work(), "itc-weight-v1", quantity(3, "itc-unit"));
        let entry = issue_credit(&assessment, "account-1", quantity(3, "itc-unit"), "entry-1");

        assert_eq!(
            reject_accounting_as_settlement(&entry, false),
            Err(SemanticViolation::AccountingPresentedAsSettlement)
        );
        assert_eq!(reject_accounting_as_settlement(&entry, true), Ok(()));
    }

    #[test]
    fn correction_must_append_lineage_instead_of_rewriting_entry() {
        let assessment = assess_work(&work(), "itc-weight-v1", quantity(3, "itc-unit"));
        let original = issue_credit(&assessment, "account-1", quantity(3, "itc-unit"), "entry-1");
        let mut replacement = original.clone();
        replacement.quantity = quantity(4, "itc-unit");
        replacement.correction.supersedes = Some(original.identity.object_id.clone());
        replacement.identity.object_id = "entry-2".into();

        assert_eq!(reject_history_rewrite(&original, &replacement), Ok(()));
        assert_eq!(replacement.correction.supersedes.as_deref(), Some("entry-1"));
    }

    #[test]
    fn valuation_does_not_change_itc_unit() {
        assert_eq!(
            reject_valuation_unit_substitution("itc-unit", "USD"),
            Err(SemanticViolation::ValuationChangesUnit)
        );
    }

    #[test]
    fn instrument_substitution_is_explicitly_rejected() {
        assert_eq!(
            reject_instrument_substitution("itc-unit", "SAP"),
            Err(SemanticViolation::InstrumentSubstitution)
        );
        assert_eq!(reject_instrument_substitution("itc-unit", "itc-unit"), Ok(()));
    }

    #[test]
    fn raw_labor_is_not_issued_itc() {
        let observation = work();
        let claimed = quantity(8, "itc-unit");

        assert_eq!(
            reject_if_labor_is_issuance(&observation, Some(&claimed)),
            ConformanceDecision::Rejected
        );
        assert_eq!(
            reject_if_labor_is_issuance(&observation, None),
            ConformanceDecision::Accepted
        );
    }

    #[test]
    fn corpus_is_complete_and_machine_stable() {
        assert_eq!(POSITIVE_CASES.len(), 9);
        assert_eq!(NEGATIVE_CASES.len(), 9);
        assert!(NEGATIVE_CASES.contains(&"itc_to_sap"));
        assert!(NEGATIVE_CASES.contains(&"itc_to_tend"));
        assert!(NEGATIVE_CASES.contains(&"itc_to_mycel"));
        assert!(NEGATIVE_CASES.contains(&"foreign_to_local_mint"));
    }
}

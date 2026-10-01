use serde::{Deserialize, Serialize};
use crate::identity_lineage::IdentityRef;
use crate::temporal_applicability::TemporalInterval;
use crate::temporal_reconciliation::{
    classify, Comparability, Compatibility, ReconciliationClass, ReconciliationResult,
};

/// An auditable, reproducible witness for a temporal reconciliation result.
///
/// This is provenance structure only. It does not adjudicate physical truth.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct TemporalReconciliationWitness {
    /// Stable engineering identity for this witness; distinct from both claims.
    pub witness_identity: IdentityRef,
    pub left_claim: IdentityRef,
    pub right_claim: IdentityRef,
    pub left_applicability: TemporalInterval,
    pub right_applicability: TemporalInterval,
    pub comparability: Comparability,
    pub compatibility: Compatibility,
    pub explicitly_superseded: bool,
    pub disputed: bool,
    pub result: ReconciliationResult,
}

impl TemporalReconciliationWitness {
    pub fn validate(&self) -> Result<(), String> {
        self.witness_identity.validate()?;
        if self.witness_identity.kind != crate::identity_lineage::IdentityKind::ReconciliationWitness {
            return Err("reconciliation witness identity must use ReconciliationWitness kind".into());
        }
        self.left_claim.validate()?;
        self.right_claim.validate()?;
        if self.left_claim == self.right_claim {
            return Err("a reconciliation witness requires two distinct claim identities".into());
        }
        let expected = classify(
            self.comparability,
            self.compatibility,
            &self.left_applicability,
            &self.right_applicability,
            self.explicitly_superseded,
            self.disputed,
        )
        .map_err(|e| e.to_string())?;
        if self.result != expected {
            return Err("stored reconciliation result does not match witness inputs".into());
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::identity_lineage::IdentityKind;
    use crate::temporal_applicability::TemporalPoint;

    fn claim(id: &str, kind: IdentityKind) -> IdentityRef {
        IdentityRef { kind, namespace: "synthetic".into(), id: id.into() }
    }

    fn interval(start: Option<i64>, end: Option<i64>) -> TemporalInterval {
        TemporalInterval {
            start: start.map(TemporalPoint),
            end: end.map(TemporalPoint),
        }
    }

    fn conflict_witness() -> TemporalReconciliationWitness {
        let witness_identity = claim("witness-1", IdentityKind::ReconciliationWitness);
        let left = claim("claim-a", IdentityKind::EvidenceRecord);
        let right = claim("claim-b", IdentityKind::EvidenceRecord);
        let left_applicability = interval(Some(0), Some(10));
        let right_applicability = interval(Some(5), Some(15));
        let result = classify(
            Comparability::Comparable,
            Compatibility::Incompatible,
            &left_applicability,
            &right_applicability,
            false,
            false,
        ).unwrap();
        TemporalReconciliationWitness {
            witness_identity,
            left_claim: left,
            right_claim: right,
            left_applicability,
            right_applicability,
            comparability: Comparability::Comparable,
            compatibility: Compatibility::Incompatible,
            explicitly_superseded: false,
            disputed: false,
            result,
        }
    }

    #[test]
    fn witness_identity_is_distinct_from_claim_identity() {
        let witness = conflict_witness();
        assert_eq!(witness.witness_identity.kind, IdentityKind::ReconciliationWitness);
        assert_ne!(witness.witness_identity, witness.left_claim);
        assert_ne!(witness.witness_identity, witness.right_claim);
    }

    #[test]
    fn wrong_witness_identity_kind_is_rejected() {
        let mut witness = conflict_witness();
        witness.witness_identity.kind = IdentityKind::EvidenceRecord;
        assert!(witness.validate().is_err());
    }

    #[test]
    fn missing_witness_identity_is_rejected() {
        let mut witness = conflict_witness();
        witness.witness_identity.id.clear();
        assert!(witness.validate().is_err());
    }

    #[test]
    fn valid_conflict_witness_reproduces_exactly() {
        assert!(conflict_witness().validate().is_ok());
        assert_eq!(conflict_witness().result.classification, ReconciliationClass::Conflicting);
    }

    #[test]
    fn tampered_result_is_rejected() {
        let mut witness = conflict_witness();
        witness.result.classification = ReconciliationClass::Sequential;
        assert!(witness.validate().is_err());
    }

    #[test]
    fn missing_identity_semantics_are_rejected() {
        let mut witness = conflict_witness();
        witness.left_claim.id.clear();
        assert!(witness.validate().is_err());
    }

    #[test]
    fn same_claim_cannot_reconcile_with_itself() {
        let mut witness = conflict_witness();
        witness.right_claim = witness.left_claim.clone();
        assert!(witness.validate().is_err());
    }

    #[test]
    fn dispute_is_preserved_without_changing_conflict_class() {
        let mut witness = conflict_witness();
        witness.disputed = true;
        witness.result.disputed = true;
        assert!(witness.validate().is_ok());
        assert_eq!(witness.result.classification, ReconciliationClass::Conflicting);
    }

    #[test]
    fn supersession_is_explicit_and_reproducible() {
        let mut witness = conflict_witness();
        witness.explicitly_superseded = true;
        witness.result = classify(
            witness.comparability,
            witness.compatibility,
            &witness.left_applicability,
            &witness.right_applicability,
            true,
            false,
        ).unwrap();
        assert!(witness.validate().is_ok());
        assert_eq!(witness.result.classification, ReconciliationClass::Superseded);
    }

    #[test]
    fn indeterminate_temporal_witness_remains_indeterminate() {
        let mut witness = conflict_witness();
        witness.right_applicability.end = None;
        witness.result = classify(
            witness.comparability,
            witness.compatibility,
            &witness.left_applicability,
            &witness.right_applicability,
            false,
            false,
        ).unwrap();
        assert!(witness.validate().is_ok());
        assert_eq!(witness.result.classification, ReconciliationClass::Indeterminate);
    }
}

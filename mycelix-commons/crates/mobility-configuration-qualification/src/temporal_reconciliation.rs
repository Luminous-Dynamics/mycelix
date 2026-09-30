//! Conservative temporal claim reconciliation; semantic/provenance only.
use serde::{Deserialize, Serialize};
use crate::temporal_applicability::TemporalInterval;

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Comparability { Comparable, Incomparable, Unknown }

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum Compatibility { Compatible, Incompatible, Unknown }

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ReconciliationClass { Coexistent, Conflicting, Sequential, Incomparable, Indeterminate, Superseded }

#[derive(Clone, Copy, Debug, Eq, PartialEq, Serialize, Deserialize)]
pub struct ReconciliationResult {
    pub classification: ReconciliationClass,
    /// A recorded dispute is metadata, not a resolution or truth judgment.
    pub disputed: bool,
}

pub fn classify(
    comparability: Comparability,
    compatibility: Compatibility,
    left: &TemporalInterval,
    right: &TemporalInterval,
    explicitly_superseded: bool,
    disputed: bool,
) -> Result<ReconciliationResult, &'static str> {
    left.validate().map_err(|_| "invalid left interval")?;
    right.validate().map_err(|_| "invalid right interval")?;
    let classification = if explicitly_superseded {
        ReconciliationClass::Superseded
    } else if comparability == Comparability::Incomparable {
        ReconciliationClass::Incomparable
    } else if comparability == Comparability::Unknown || compatibility == Compatibility::Unknown {
        ReconciliationClass::Indeterminate
    } else if compatibility == Compatibility::Compatible {
        ReconciliationClass::Coexistent
    } else {
        match left.overlaps(right) {
            Some(true) => ReconciliationClass::Conflicting,
            Some(false) => ReconciliationClass::Sequential,
            None => ReconciliationClass::Indeterminate,
        }
    };
    Ok(ReconciliationResult {
        classification,
        disputed: disputed && classification == ReconciliationClass::Conflicting,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::temporal_applicability::TemporalPoint;
    fn interval(a: Option<i64>, b: Option<i64>) -> TemporalInterval {
        TemporalInterval { start: a.map(TemporalPoint), end: b.map(TemporalPoint) }
    }
    #[test] fn compatible_overlap_is_coexistent() {
        assert_eq!(classify(Comparability::Comparable, Compatibility::Compatible,&interval(Some(0),Some(5)),&interval(Some(3),Some(8)),false,false).unwrap().classification,ReconciliationClass::Coexistent);
    }
    #[test] fn incompatible_overlap_is_conflict() {
        assert_eq!(classify(Comparability::Comparable,Compatibility::Incompatible,&interval(Some(0),Some(5)),&interval(Some(3),Some(8)),false,false).unwrap().classification,ReconciliationClass::Conflicting);
    }
    #[test] fn disjoint_claims_are_sequential() {
        assert_eq!(classify(Comparability::Comparable,Compatibility::Incompatible,&interval(Some(0),Some(2)),&interval(Some(3),Some(5)),false,false).unwrap().classification,ReconciliationClass::Sequential);
    }
    #[test] fn unknown_bounds_are_indeterminate() {
        assert_eq!(classify(Comparability::Comparable,Compatibility::Incompatible,&interval(Some(0),None),&interval(Some(3),Some(5)),false,false).unwrap().classification,ReconciliationClass::Indeterminate);
    }
    #[test] fn incomparable_precedes_compatibility() {
        assert_eq!(classify(Comparability::Incomparable,Compatibility::Incompatible,&interval(Some(0),Some(5)),&interval(Some(3),Some(8)),false,false).unwrap().classification,ReconciliationClass::Incomparable);
    }
    #[test] fn unknown_comparability_is_indeterminate() {
        assert_eq!(classify(Comparability::Unknown,Compatibility::Incompatible,&interval(Some(0),Some(5)),&interval(Some(3),Some(8)),false,false).unwrap().classification,ReconciliationClass::Indeterminate);
    }
    #[test] fn unknown_compatibility_is_indeterminate() {
        assert_eq!(classify(Comparability::Comparable,Compatibility::Unknown,&interval(Some(0),Some(5)),&interval(Some(3),Some(8)),false,false).unwrap().classification,ReconciliationClass::Indeterminate);
    }
    #[test] fn explicit_supersession_preserves_history() {
        assert_eq!(classify(Comparability::Comparable,Compatibility::Incompatible,&interval(Some(0),Some(5)),&interval(Some(3),Some(8)),true,false).unwrap().classification,ReconciliationClass::Superseded);
    }
    #[test] fn dispute_is_orthogonal_to_conflict() {
        let r=classify(Comparability::Comparable,Compatibility::Incompatible,&interval(Some(0),Some(5)),&interval(Some(3),Some(8)),false,true).unwrap();
        assert_eq!(r.classification,ReconciliationClass::Conflicting); assert!(r.disputed);
    }
    #[test] fn compatible_unknown_bounds_remain_coexistent() {
        assert_eq!(classify(Comparability::Comparable,Compatibility::Compatible,&interval(None,None),&interval(Some(3),Some(8)),false,false).unwrap().classification,ReconciliationClass::Coexistent);
    }
    #[test] fn invalid_interval_rejected() {
        assert!(classify(Comparability::Comparable,Compatibility::Incompatible,&interval(Some(5),Some(1)),&interval(Some(0),Some(2)),false,false).is_err());
    }
}

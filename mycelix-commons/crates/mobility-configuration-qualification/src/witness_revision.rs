use crate::identity_lineage::{IdentityKind, IdentityRef, LineageEdge, LineageRelation};
use crate::temporal_reconciliation_witness::TemporalReconciliationWitness;

/// Validate a proposed witness revision without treating identity as proof of truth.
///
/// A witness identity is immutable by meaning: the same identity may not be
/// reused for a materially different witness payload. A changed witness gets
/// a new identity and must explicitly supersede its predecessor.
pub fn validate_transition(
    previous: &TemporalReconciliationWitness,
    current: &TemporalReconciliationWitness,
    supersession: Option<&LineageEdge>,
) -> Result<(), String> {
    previous.validate()?;
    current.validate()?;

    if previous.witness_identity.kind != IdentityKind::ReconciliationWitness
        || current.witness_identity.kind != IdentityKind::ReconciliationWitness
    {
        return Err("witness revisions require reconciliation_witness identities".into());
    }

    if previous.witness_identity == current.witness_identity {
        if previous == current {
            return Ok(());
        }
        return Err(
            "reconciliation witness identity is immutable by meaning; changed payload requires a new identity"
                .into(),
        );
    }

    let supersession = supersession.ok_or_else(|| {
        "changed witness identity requires explicit Supersedes lineage".to_string()
    })?;

    supersession
        .validate()
        .map_err(|e| format!("invalid witness supersession lineage: {e}"))?;

    if supersession.relation != LineageRelation::Supersedes
        || supersession.source != current.witness_identity
        || supersession.target != previous.witness_identity
    {
        return Err(
            "witness supersession must point from the successor identity to the predecessor identity"
                .into(),
        );
    }

    Ok(())
}

/// Validate that a projection remains bound to the exact historical witness
/// it was created from. A successor witness cannot silently retarget it.
pub fn validate_projection_binding(
    projection_witness: &IdentityRef,
    witness: &TemporalReconciliationWitness,
) -> Result<(), String> {
    witness.validate()?;
    projection_witness.validate()?;
    if projection_witness != &witness.witness_identity {
        return Err("historical projection is bound to a different witness identity".into());
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::identity_lineage::IdentityKind;
    use crate::temporal_applicability::{TemporalInterval, TemporalPoint};
    use crate::temporal_reconciliation::{Comparability, Compatibility, classify};

    fn id(kind: IdentityKind, value: &str) -> IdentityRef {
        IdentityRef {
            kind,
            namespace: "synthetic".into(),
            id: value.into(),
        }
    }

    fn witness(identity: &str) -> TemporalReconciliationWitness {
        let left_applicability = TemporalInterval {
            start: Some(TemporalPoint(0)),
            end: Some(TemporalPoint(10)),
        };
        let right_applicability = TemporalInterval {
            start: Some(TemporalPoint(5)),
            end: Some(TemporalPoint(15)),
        };
        let result = classify(
            Comparability::Comparable,
            Compatibility::Incompatible,
            &left_applicability,
            &right_applicability,
            false,
            false,
        )
        .unwrap();

        TemporalReconciliationWitness {
            witness_identity: id(IdentityKind::ReconciliationWitness, identity),
            left_claim: id(IdentityKind::EvidenceRecord, "claim-a"),
            right_claim: id(IdentityKind::EvidenceRecord, "claim-b"),
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
    fn identical_payload_with_same_identity_is_not_a_revision() {
        let a = witness("w1");
        assert!(validate_transition(&a, &a, None).is_ok());
    }

    #[test]
    fn changed_claim_pair_cannot_reuse_identity() {
        let previous = witness("w1");
        let mut current = previous.clone();
        current.right_claim = id(IdentityKind::EvidenceRecord, "claim-c");
        assert!(validate_transition(&previous, &current, None).is_err());
    }

    #[test]
    fn changed_applicability_cannot_reuse_identity() {
        let previous = witness("w1");
        let mut current = previous.clone();
        current.right_applicability.end = Some(TemporalPoint(20));
        current.result = classify(
            current.comparability,
            current.compatibility,
            &current.left_applicability,
            &current.right_applicability,
            current.explicitly_superseded,
            current.disputed,
        )
        .unwrap();
        assert!(validate_transition(&previous, &current, None).is_err());
    }

    #[test]
    fn changed_result_inputs_cannot_reuse_identity() {
        let previous = witness("w1");
        let mut current = previous.clone();
        current.compatibility = Compatibility::Compatible;
        current.result = classify(
            current.comparability,
            current.compatibility,
            &current.left_applicability,
            &current.right_applicability,
            current.explicitly_superseded,
            current.disputed,
        )
        .unwrap();
        assert!(validate_transition(&previous, &current, None).is_err());
    }

    #[test]
    fn changed_witness_requires_new_identity_and_supersession() {
        let previous = witness("w1");
        let mut current = previous.clone();
        current.witness_identity = id(IdentityKind::ReconciliationWitness, "w2");
        current.right_applicability.end = Some(TemporalPoint(20));
        current.result = classify(
            current.comparability,
            current.compatibility,
            &current.left_applicability,
            &current.right_applicability,
            current.explicitly_superseded,
            current.disputed,
        )
        .unwrap();
        let lineage = LineageEdge {
            relation: LineageRelation::Supersedes,
            source: current.witness_identity.clone(),
            target: previous.witness_identity.clone(),
        };
        assert!(validate_transition(&previous, &current, Some(&lineage)).is_ok());
    }

    #[test]
    fn mismatched_supersession_lineage_is_rejected() {
        let previous = witness("w1");
        let current = witness("w2");
        let wrong = LineageEdge {
            relation: LineageRelation::Supersedes,
            source: previous.witness_identity.clone(),
            target: current.witness_identity.clone(),
        };
        assert!(validate_transition(&previous, &current, Some(&wrong)).is_err());
    }

    #[test]
    fn successor_cannot_retarget_predecessor_projection() {
        let previous = witness("w1");
        let successor = witness("w2");
        assert!(validate_projection_binding(&previous.witness_identity, &previous).is_ok());
        assert!(validate_projection_binding(&previous.witness_identity, &successor).is_err());
    }

    #[test]
    fn predecessor_remains_valid_after_successor_exists() {
        let previous = witness("w1");
        let mut successor = witness("w2");
        successor.right_applicability.end = Some(TemporalPoint(20));
        successor.result = classify(
            successor.comparability,
            successor.compatibility,
            &successor.left_applicability,
            &successor.right_applicability,
            successor.explicitly_superseded,
            successor.disputed,
        )
        .unwrap();

        assert!(previous.validate().is_ok());
        let lineage = LineageEdge {
            relation: LineageRelation::Supersedes,
            source: successor.witness_identity.clone(),
            target: previous.witness_identity.clone(),
        };
        assert!(validate_transition(&previous, &successor, Some(&lineage)).is_ok());
        assert!(previous.validate().is_ok());
    }
}

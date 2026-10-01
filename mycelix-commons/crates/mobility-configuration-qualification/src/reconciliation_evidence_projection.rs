use crate::{
    identity_lineage::{IdentityKind, IdentityRef},
    temporal_reconciliation_witness::TemporalReconciliationWitness,
    ConflictDisposition, EpistemicDisposition, EvidenceState, LifecycleDisposition,
};
use crate::temporal_reconciliation::ReconciliationClass;
use serde::{Deserialize, Serialize};

/// Explicit, auditable projection from a temporal reconciliation witness into
/// the orthogonal EvidenceState tuple.
///
/// This layer deliberately refuses to turn reconciliation into engineering
/// truth. In particular, a conflicting witness never automatically becomes
/// EpistemicDisposition::Contradicted.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub enum ReconciliationEvidenceProjection {
    ConflictReferenceOnly {
        witness_ref: IdentityRef,
    },
    LifecycleSupersession {
        witness_ref: IdentityRef,
    },
    IndeterminateReference {
        witness_ref: IdentityRef,
        unresolved_dependency_reference: Option<String>,
    },
    NoEpistemicPromotion {
        witness_ref: IdentityRef,
    },
}

impl ReconciliationEvidenceProjection {
    pub fn validate(
        &self,
        witness: &TemporalReconciliationWitness,
        base: &EvidenceState,
    ) -> Result<(), String> {
        witness.validate()?;
        base.validate()?;

        let witness_ref = self.witness_ref();
        witness_ref.validate()?;
        if witness_ref.kind != IdentityKind::ReconciliationWitness {
            return Err("reconciliation projection reference must use ReconciliationWitness kind".into());
        }
        if witness_ref != &witness.witness_identity {
            return Err("reconciliation projection reference does not identify the supplied witness".into());
        }

        match (&self, witness.result.classification) {
            (Self::ConflictReferenceOnly { .. }, ReconciliationClass::Conflicting) => {}
            (Self::LifecycleSupersession { .. }, ReconciliationClass::Superseded) => {}
            (Self::IndeterminateReference { .. }, ReconciliationClass::Indeterminate) => {}
            (Self::NoEpistemicPromotion { .. }, ReconciliationClass::Coexistent)
            | (Self::NoEpistemicPromotion { .. }, ReconciliationClass::Sequential)
            | (Self::NoEpistemicPromotion { .. }, ReconciliationClass::Incomparable) => {}
            _ => return Err("projection effect is incompatible with reconciliation classification".into()),
        }

        if matches!(self, Self::ConflictReferenceOnly { .. })
            && base.epistemic_disposition == EpistemicDisposition::Contradicted
            && base.contradiction_reference.is_none()
        {
            return Err("projection cannot repair an invalid contradiction state".into());
        }

        if let Self::IndeterminateReference {
            unresolved_dependency_reference: Some(reference),
            ..
        } = self
        {
            if reference.trim().is_empty() {
                return Err("unresolved dependency reference cannot be empty".into());
            }
        }

        Ok(())
    }

    /// Apply only the explicit effect represented by this projection.
    ///
    /// No branch changes authority provenance or evidence modality, and no
    /// reconciliation class promotes itself to engineering correctness.
    pub fn apply(&self, witness: &TemporalReconciliationWitness, base: &EvidenceState) -> Result<EvidenceState, String> {
        self.validate(witness, base)?;
        let mut state = base.clone();

        match self {
            Self::ConflictReferenceOnly { witness_ref } => {
                state.conflict_reference = Some(witness_ref.id.clone());
                state.conflict_disposition = if witness.result.disputed {
                    ConflictDisposition::Disputed
                } else {
                    ConflictDisposition::Uncontested
                };
            }
            Self::LifecycleSupersession { .. } => {
                state.lifecycle_disposition = LifecycleDisposition::Superseded;
            }
            Self::IndeterminateReference {
                unresolved_dependency_reference,
                ..
            } => {
                if let Some(reference) = unresolved_dependency_reference {
                    state.epistemic_disposition = EpistemicDisposition::Unresolved;
                    state.unresolved_dependency_reference = Some(reference.clone());
                } else {
                    state.epistemic_disposition = EpistemicDisposition::Indeterminate;
                }
            }
            Self::NoEpistemicPromotion { .. } => {}
        }

        state.validate()?;
        Ok(state)
    }

    fn witness_ref(&self) -> &IdentityRef {
        match self {
            Self::ConflictReferenceOnly { witness_ref }
            | Self::LifecycleSupersession { witness_ref }
            | Self::IndeterminateReference { witness_ref, .. }
            | Self::NoEpistemicPromotion { witness_ref } => witness_ref,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::identity_lineage::{IdentityKind, IdentityRef};
    use crate::temporal_applicability::{TemporalInterval, TemporalPoint};
    use crate::temporal_reconciliation::{classify, Comparability, Compatibility};

    fn claim(id: &str) -> IdentityRef {
        IdentityRef {
            kind: IdentityKind::EvidenceRecord,
            namespace: "synthetic".into(),
            id: id.into(),
        }
    }

    fn witness_identity(id: &str) -> IdentityRef {
        IdentityRef {
            kind: IdentityKind::ReconciliationWitness,
            namespace: "synthetic".into(),
            id: id.into(),
        }
    }

    fn interval(start: Option<i64>, end: Option<i64>) -> TemporalInterval {
        TemporalInterval {
            start: start.map(TemporalPoint),
            end: end.map(TemporalPoint),
        }
    }

    fn conflict_witness() -> TemporalReconciliationWitness {
        let left_applicability = interval(Some(0), Some(10));
        let right_applicability = interval(Some(5), Some(15));
        let result = classify(
            Comparability::Comparable,
            Compatibility::Incompatible,
            &left_applicability,
            &right_applicability,
            false,
            true,
        )
        .unwrap();

        TemporalReconciliationWitness {
            witness_identity: witness_identity("witness-1"),
            left_claim: claim("claim-a"),
            right_claim: claim("claim-b"),
            left_applicability,
            right_applicability,
            comparability: Comparability::Comparable,
            compatibility: Compatibility::Incompatible,
            explicitly_superseded: false,
            disputed: true,
            result,
        }
    }

    fn base_state() -> EvidenceState {
        EvidenceState {
            epistemic_disposition: EpistemicDisposition::Supported,
            lifecycle_disposition: LifecycleDisposition::Current,
            conflict_disposition: ConflictDisposition::Uncontested,
            authority_provenance: crate::AuthorityProvenance::Commons,
            evidence_modality: crate::EvidenceModality::Interpretation,
            contradiction_reference: None,
            conflict_reference: None,
            unresolved_dependency_reference: None,
            external_authority_reference: None,
        }
    }

    #[test]
    fn conflicting_witness_projects_to_conflict_reference_not_contradiction() {
        let witness = conflict_witness();
        let projection = ReconciliationEvidenceProjection::ConflictReferenceOnly {
            witness_ref: witness_identity("witness-1"),
        };
        let projected = projection.apply(&witness, &base_state()).unwrap();
        assert_eq!(projected.conflict_reference.as_deref(), Some("witness-1"));
        assert_eq!(projected.conflict_disposition, ConflictDisposition::Disputed);
        assert_eq!(projected.epistemic_disposition, EpistemicDisposition::Supported);
        assert!(projected.contradiction_reference.is_none());
    }

    #[test]
    fn conflicting_witness_cannot_use_lifecycle_or_indeterminate_projection() {
        let witness = conflict_witness();
        assert!(ReconciliationEvidenceProjection::LifecycleSupersession {
            witness_ref: witness_identity("witness-1")
        }
        .validate(&witness, &base_state())
        .is_err());
        assert!(ReconciliationEvidenceProjection::IndeterminateReference {
            witness_ref: witness_identity("witness-1"),
            unresolved_dependency_reference: None
        }
        .validate(&witness, &base_state())
        .is_err());
    }

    #[test]
    fn superseded_witness_changes_lifecycle_only() {
        let mut witness = conflict_witness();
        witness.explicitly_superseded = true;
        witness.disputed = false;
        witness.result = classify(
            witness.comparability,
            witness.compatibility,
            &witness.left_applicability,
            &witness.right_applicability,
            true,
            false,
        )
        .unwrap();

        let projection = ReconciliationEvidenceProjection::LifecycleSupersession {
            witness_ref: witness_identity("witness-1"),
        };
        let projected = projection.apply(&witness, &base_state()).unwrap();
        assert_eq!(projected.lifecycle_disposition, LifecycleDisposition::Superseded);
        assert_eq!(projected.epistemic_disposition, EpistemicDisposition::Supported);
    }

    #[test]
    fn indeterminate_witness_never_becomes_contradicted() {
        let mut witness = conflict_witness();
        witness.disputed = false;
        witness.right_applicability.end = None;
        witness.result = classify(
            witness.comparability,
            witness.compatibility,
            &witness.left_applicability,
            &witness.right_applicability,
            false,
            false,
        )
        .unwrap();

        let projection = ReconciliationEvidenceProjection::IndeterminateReference {
            witness_ref: witness_identity("witness-1"),
            unresolved_dependency_reference: None,
        };
        let projected = projection.apply(&witness, &base_state()).unwrap();
        assert_eq!(projected.epistemic_disposition, EpistemicDisposition::Indeterminate);
        assert_ne!(projected.epistemic_disposition, EpistemicDisposition::Contradicted);
    }

    #[test]
    fn indeterminate_witness_can_explicitly_record_unresolved_dependency() {
        let mut witness = conflict_witness();
        witness.disputed = false;
        witness.right_applicability.end = None;
        witness.result = classify(
            witness.comparability,
            witness.compatibility,
            &witness.left_applicability,
            &witness.right_applicability,
            false,
            false,
        )
        .unwrap();

        let projection = ReconciliationEvidenceProjection::IndeterminateReference {
            witness_ref: witness_identity("witness-1"),
            unresolved_dependency_reference: Some("dependency-1".into()),
        };
        let projected = projection.apply(&witness, &base_state()).unwrap();
        assert_eq!(projected.epistemic_disposition, EpistemicDisposition::Unresolved);
        assert_eq!(
            projected.unresolved_dependency_reference.as_deref(),
            Some("dependency-1")
        );
    }

    #[test]
    fn coexistent_does_not_promote_to_supported() {
        let mut base = base_state();
        base.epistemic_disposition = EpistemicDisposition::Indeterminate;
        let mut witness = conflict_witness();
        witness.disputed = false;
        witness.compatibility = Compatibility::Compatible;
        witness.result = classify(
            witness.comparability,
            witness.compatibility,
            &witness.left_applicability,
            &witness.right_applicability,
            false,
            false,
        )
        .unwrap();

        let projection = ReconciliationEvidenceProjection::NoEpistemicPromotion {
            witness_ref: witness_identity("witness-1"),
        };
        let projected = projection.apply(&witness, &base).unwrap();
        assert_eq!(projected.epistemic_disposition, EpistemicDisposition::Indeterminate);
        assert_eq!(projected.conflict_reference, None);
    }

    #[test]
    fn incompatible_sequential_does_not_imply_causality() {
        let mut base = base_state();
        base.epistemic_disposition = EpistemicDisposition::Indeterminate;
        let mut witness = conflict_witness();
        witness.disputed = false;
        witness.right_applicability = interval(Some(20), Some(30));
        witness.result = classify(
            witness.comparability,
            witness.compatibility,
            &witness.left_applicability,
            &witness.right_applicability,
            false,
            false,
        )
        .unwrap();

        let projection = ReconciliationEvidenceProjection::NoEpistemicPromotion {
            witness_ref: witness_identity("witness-1"),
        };
        assert_eq!(witness.result.classification, ReconciliationClass::Sequential);
        assert!(projection.apply(&witness, &base).is_ok());
    }

    #[test]
    fn incomparable_does_not_become_disputed() {
        let mut base = base_state();
        base.epistemic_disposition = EpistemicDisposition::Indeterminate;
        let mut witness = conflict_witness();
        witness.disputed = false;
        witness.comparability = Comparability::Incomparable;
        witness.result = classify(
            witness.comparability,
            witness.compatibility,
            &witness.left_applicability,
            &witness.right_applicability,
            false,
            false,
        )
        .unwrap();

        let projection = ReconciliationEvidenceProjection::NoEpistemicPromotion {
            witness_ref: witness_identity("witness-1"),
        };
        let projected = projection.apply(&witness, &base).unwrap();
        assert_eq!(projected.conflict_disposition, ConflictDisposition::Uncontested);
        assert_eq!(projected.epistemic_disposition, EpistemicDisposition::Indeterminate);
    }

    #[test]
    fn projection_preserves_external_authority_and_modality() {
        let witness = conflict_witness();
        let mut base = base_state();
        base.authority_provenance = crate::AuthorityProvenance::External;
        base.external_authority_reference = Some("authority-1".into());
        base.evidence_modality = crate::EvidenceModality::Measurement;
        let projection = ReconciliationEvidenceProjection::ConflictReferenceOnly {
            witness_ref: witness_identity("witness-1"),
        };
        let projected = projection.apply(&witness, &base).unwrap();
        assert_eq!(projected.authority_provenance, crate::AuthorityProvenance::External);
        assert_eq!(projected.external_authority_reference.as_deref(), Some("authority-1"));
        assert_eq!(projected.evidence_modality, crate::EvidenceModality::Measurement);
    }

    #[test]
    fn wrong_witness_identity_is_rejected() {
        let witness = conflict_witness();
        let projection = ReconciliationEvidenceProjection::ConflictReferenceOnly {
            witness_ref: witness_identity("different-witness"),
        };
        assert!(projection.validate(&witness, &base_state()).is_err());
    }

    #[test]
    fn claim_identity_cannot_be_used_as_witness_reference() {
        let witness = conflict_witness();
        let projection = ReconciliationEvidenceProjection::ConflictReferenceOnly {
            witness_ref: claim("claim-a"),
        };
        assert!(projection.validate(&witness, &base_state()).is_err());
    }

    #[test]
    fn empty_witness_reference_is_rejected() {
        let witness = conflict_witness();
        let projection = ReconciliationEvidenceProjection::ConflictReferenceOnly {
            witness_ref: IdentityRef { kind: IdentityKind::ReconciliationWitness, namespace: "synthetic".into(), id: " ".into() },
        };
        assert!(projection.validate(&witness, &base_state()).is_err());
    }
}

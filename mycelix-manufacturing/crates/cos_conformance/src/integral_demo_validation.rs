//! D6K validation-versus-conclusion semantic boundary.
//!
//! Validation answers structural questions about a declared artifact.
//! Conclusion status answers what the reference model explicitly records about
//! support or disagreement. Neither is a truth oracle.
//!
//! Claim ceiling: ReferenceModelOnly.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValidationStatus {
    StructurallyValid,
    Invalid,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum EvidenceStatus {
    Unbound,
    Bound,
    Conflicting,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ConclusionStatus {
    Unresolved,
    Supported,
    Disputed,
    Rejected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ValidationRecord {
    pub artifact_id: &'static str,
    pub validation: ValidationStatus,
    pub evidence: EvidenceStatus,
    pub conclusion: ConclusionStatus,
    pub human_disposition: Option<bool>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ValidationError {
    EmptyIdentity,
    InvalidStructure,
    ConclusionWithoutEvidence,
    SupportedWithoutEvidence,
    ConflictingEvidenceCannotBeDeclaredSupported,
    DispositionWithoutConclusion,
}

pub fn validate_record(record: &ValidationRecord) -> Result<(), ValidationError> {
    if record.artifact_id.trim().is_empty() {
        return Err(ValidationError::EmptyIdentity);
    }
    if record.validation != ValidationStatus::StructurallyValid {
        return Err(ValidationError::InvalidStructure);
    }

    match record.conclusion {
        ConclusionStatus::Supported => {
            if record.evidence == EvidenceStatus::Unbound {
                return Err(ValidationError::SupportedWithoutEvidence);
            }
            if record.evidence == EvidenceStatus::Conflicting {
                return Err(ValidationError::ConflictingEvidenceCannotBeDeclaredSupported);
            }
        }
        ConclusionStatus::Rejected => {
            if record.evidence == EvidenceStatus::Unbound {
                return Err(ValidationError::ConclusionWithoutEvidence);
            }
        }
        ConclusionStatus::Disputed => {
            if record.evidence != EvidenceStatus::Conflicting {
                return Err(ValidationError::ConclusionWithoutEvidence);
            }
        }
        ConclusionStatus::Unresolved => {}
    }

    if record.human_disposition.is_some() && record.conclusion == ConclusionStatus::Unresolved {
        return Err(ValidationError::DispositionWithoutConclusion);
    }

    Ok(())
}

/// A structural validation result is never itself a conclusion about truth.
pub fn validation_does_not_conclude(record: &ValidationRecord) -> bool {
    record.validation == ValidationStatus::StructurallyValid
        && record.conclusion == ConclusionStatus::Unresolved
}

/// Explicit human disposition remains distinct from evidence support.
pub fn disposition_is_not_evidence(record: &ValidationRecord) -> bool {
    record.human_disposition.is_some()
        && record.evidence != EvidenceStatus::Unbound
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_unresolved() -> ValidationRecord {
        ValidationRecord {
            artifact_id: "artifact-1",
            validation: ValidationStatus::StructurallyValid,
            evidence: EvidenceStatus::Bound,
            conclusion: ConclusionStatus::Unresolved,
            human_disposition: None,
        }
    }

    #[test]
    fn structural_validity_does_not_create_a_conclusion() {
        let record = valid_unresolved();
        assert!(validate_record(&record).is_ok());
        assert!(validation_does_not_conclude(&record));
    }

    #[test]
    fn supported_requires_bound_non_conflicting_evidence() {
        let mut record = valid_unresolved();
        record.conclusion = ConclusionStatus::Supported;
        assert!(validate_record(&record).is_ok());

        record.evidence = EvidenceStatus::Unbound;
        assert_eq!(validate_record(&record), Err(ValidationError::SupportedWithoutEvidence));

        record.evidence = EvidenceStatus::Conflicting;
        assert_eq!(
            validate_record(&record),
            Err(ValidationError::ConflictingEvidenceCannotBeDeclaredSupported)
        );
    }

    #[test]
    fn disputed_requires_explicit_conflicting_evidence() {
        let mut record = valid_unresolved();
        record.conclusion = ConclusionStatus::Disputed;
        assert_eq!(
            validate_record(&record),
            Err(ValidationError::ConclusionWithoutEvidence)
        );

        record.evidence = EvidenceStatus::Conflicting;
        assert!(validate_record(&record).is_ok());
    }

    #[test]
    fn rejection_does_not_require_the_model_to_claim_truth() {
        let mut record = valid_unresolved();
        record.conclusion = ConclusionStatus::Rejected;
        assert!(validate_record(&record).is_ok());
    }

    #[test]
    fn human_disposition_does_not_substitute_for_evidence() {
        let mut record = valid_unresolved();
        record.conclusion = ConclusionStatus::Supported;
        record.human_disposition = Some(true);
        assert!(validate_record(&record).is_ok());
        assert!(disposition_is_not_evidence(&record));
    }

    #[test]
    fn disposition_cannot_resolve_an_unresolved_record_implicitly() {
        let mut record = valid_unresolved();
        record.human_disposition = Some(true);
        assert_eq!(
            validate_record(&record),
            Err(ValidationError::DispositionWithoutConclusion)
        );
    }

    #[test]
    fn invalid_structure_cannot_be_rescued_by_conclusion() {
        let mut record = valid_unresolved();
        record.validation = ValidationStatus::Invalid;
        record.conclusion = ConclusionStatus::Supported;
        assert_eq!(validate_record(&record), Err(ValidationError::InvalidStructure));
    }
}

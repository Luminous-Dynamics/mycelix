use crate::identity_lineage::IdentityRef;

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QualificationStatus<T> {
    Complete(T),
    Unresolved {
        missing: Vec<IdentityRef>,
        partial: T,
    },
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QualificationValidationError {
    Structural { reason: String },
}

pub type QualificationOutcome<T> =
    Result<QualificationStatus<T>, QualificationValidationError>;

impl<T> QualificationStatus<T> {
    pub fn map<U>(self, map: impl FnOnce(T) -> U) -> QualificationStatus<U> {
        match self {
            Self::Complete(value) => QualificationStatus::Complete(map(value)),
            Self::Unresolved { missing, partial } => QualificationStatus::Unresolved {
                missing,
                partial: map(partial),
            },
        }
    }

    pub fn is_complete(&self) -> bool {
        matches!(self, Self::Complete(_))
    }

    pub fn missing(&self) -> &[IdentityRef] {
        match self {
            Self::Complete(_) => &[],
            Self::Unresolved { missing, .. } => missing,
        }
    }

}

impl QualificationValidationError {
    pub fn structural(reason: impl Into<String>) -> Self {
        Self::Structural {
            reason: reason.into(),
        }
    }
}

impl std::fmt::Display for QualificationValidationError {
    fn fmt(&self, formatter: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            Self::Structural { reason } => formatter.write_str(reason),
        }
    }
}

impl std::error::Error for QualificationValidationError {}

impl From<String> for QualificationValidationError {
    fn from(reason: String) -> Self {
        Self::structural(reason)
    }
}

impl From<&str> for QualificationValidationError {
    fn from(reason: &str) -> Self {
        Self::structural(reason)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::identity_lineage::IdentityKind;

    fn id(value: &str) -> IdentityRef {
        IdentityRef {
            kind: IdentityKind::ReconciliationWitness,
            namespace: "mobility".into(),
            id: value.into(),
        }
    }

    #[test]
    fn complete_has_no_missing_dependencies() {
        let status = QualificationStatus::Complete(7u8);
        assert!(status.is_complete());
        assert!(status.missing().is_empty());
    }

    #[test]
    fn unresolved_retains_missing_dependencies_and_partial_state() {
        let missing = id("missing");
        let status = QualificationStatus::Unresolved {
            missing: vec![missing.clone()],
            partial: 7u8,
        };

        assert!(!status.is_complete());
        assert_eq!(status.missing(), &[missing]);
    }

    #[test]
    fn map_preserves_resolution_state() {
        let complete = QualificationStatus::Complete(2u8).map(|value| value * 2);
        assert_eq!(complete, QualificationStatus::Complete(4u8));

        let unresolved = QualificationStatus::Unresolved {
            missing: vec![id("missing")],
            partial: 3u8,
        }
        .map(|value| value * 2);
        assert_eq!(
            unresolved,
            QualificationStatus::Unresolved {
                missing: vec![id("missing")],
                partial: 6u8,
            }
        );
    }

    #[test]
    fn structural_error_is_machine_checkable() {
        let error = QualificationValidationError::structural("contradiction");
        assert_eq!(
            error,
            QualificationValidationError::Structural {
                reason: "contradiction".into(),
            }
        );
        assert_eq!(error.to_string(), "contradiction");
    }
}

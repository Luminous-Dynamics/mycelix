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
    pub fn unresolved(mut missing: Vec<IdentityRef>, partial: T) -> Self {
        missing.sort_by(|left, right| {
            (&left.namespace, &left.kind, &left.id)
                .cmp(&(&right.namespace, &right.kind, &right.id))
        });
        missing.dedup();
        Self::Unresolved { missing, partial }
    }

    /// Adds unavailable dependencies without discarding the partial
    /// qualification payload. This is the compositional boundary for
    /// independently qualified dependency paths.
    ///
    /// A complete status becomes unresolved when any additional dependency
    /// is unavailable. Existing unresolved dependencies and newly added
    /// dependencies are canonicalized into one deterministic set.
    pub fn require_missing(
        self,
        additional: impl IntoIterator<Item = IdentityRef>,
    ) -> Self {
        let additional: Vec<_> = additional.into_iter().collect();

        match self {
            Self::Complete(partial) => {
                if additional.is_empty() {
                    Self::Complete(partial)
                } else {
                    Self::unresolved(additional, partial)
                }
            }
            Self::Unresolved {
                mut missing,
                partial,
            } => {
                missing.extend(additional);
                Self::unresolved(missing, partial)
            }
        }
    }

    pub fn normalize_missing(&mut self) {
        if let Self::Unresolved { missing, .. } = self {
            missing.sort_by(|left, right| {
                (&left.namespace, &left.kind, &left.id)
                    .cmp(&(&right.namespace, &right.kind, &right.id))
            });
            missing.dedup();
        }
    }

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

impl<T> QualificationOutcome<T> {
    /// Adds unavailable dependencies to a successful qualification result.
    ///
    /// Structural validation errors are preserved unchanged; only the
    /// Complete/Unresolved status is enriched with additional missing
    /// addressable dependencies.
    pub fn require_missing(
        self,
        additional: impl IntoIterator<Item = IdentityRef>,
    ) -> Self {
        self.map(|status| status.require_missing(additional))
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
    fn unresolved_normalizes_missing_dependency_set() {
        let first = id("z");
        let second = id("a");
        let status = QualificationStatus::unresolved(
            vec![first.clone(), second.clone(), first.clone()],
            7u8,
        );

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![second, first],
                partial: 7u8,
            }
        );
    }

    #[test]
    fn require_missing_promotes_complete_and_preserves_partial() {
        let status = QualificationStatus::Complete(7u8)
            .require_missing(vec![id("b"), id("a"), id("b")]);

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![id("a"), id("b")],
                partial: 7u8,
            }
        );
    }

    #[test]
    fn require_missing_unions_existing_and_new_dependencies() {
        let status = QualificationStatus::Unresolved {
            missing: vec![id("z")],
            partial: 9u8,
        }
        .require_missing(vec![id("a"), id("z"), id("m")]);

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![id("a"), id("m"), id("z")],
                partial: 9u8,
            }
        );
    }

    #[test]
    fn require_missing_empty_is_noop() {
        let status = QualificationStatus::Complete(7u8).require_missing(Vec::new());
        assert_eq!(status, QualificationStatus::Complete(7u8));
    }

    #[test]
    fn require_missing_recanonicalizes_existing_unresolved_dependencies() {
        let status = QualificationStatus::Unresolved {
            missing: vec![id("z"), id("a"), id("z")],
            partial: 7u8,
        }
        .require_missing(Vec::new());

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![id("a"), id("z")],
                partial: 7u8,
            }
        );
    }

    #[test]
    fn require_missing_is_order_independent_and_idempotent() {
        let first = id("a");
        let second = id("b");

        let left = QualificationStatus::Complete(7u8)
            .require_missing(vec![first.clone(), second.clone()])
            .require_missing(vec![first.clone()]);
        let right = QualificationStatus::Complete(7u8)
            .require_missing(vec![second, first]);

        assert_eq!(left, right);
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
    fn outcome_require_missing_preserves_structural_errors() {
        let error = QualificationValidationError::structural("contradiction");
        let result: QualificationOutcome<u8> = Err(error.clone())
            .require_missing(vec![id("ignored")]);

        assert_eq!(result, Err(error));
    }

    #[test]
    fn outcome_require_missing_promotes_success_to_unresolved() {
        let result: QualificationOutcome<u8> = Ok(QualificationStatus::Complete(7))
            .require_missing(vec![id("missing")]);

        assert_eq!(
            result,
            Ok(QualificationStatus::Unresolved {
                missing: vec![id("missing")],
                partial: 7,
            })
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

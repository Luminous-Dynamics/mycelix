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

/// Runtime-neutral three-outcome boundary for protocol adapters.
///
/// This mirrors the semantic distinction needed by distributed validation:
/// valid completion, definitive structural invalidity, and unresolved
/// addressable dependencies. It contains no Holochain-specific types so the
/// pure qualification crate remains portable.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum QualificationDecision<T> {
    Valid(T),
    Invalid { reason: String },
    Unresolved {
        missing: Vec<IdentityRef>,
        partial: T,
    },
}

impl<T> From<QualificationOutcome<T>> for QualificationDecision<T> {
    fn from(outcome: QualificationOutcome<T>) -> Self {
        match outcome {
            Ok(QualificationStatus::Complete(value)) => Self::Valid(value),
            Ok(QualificationStatus::Unresolved { missing, partial }) => Self::Unresolved {
                missing: canonicalize_missing(missing),
                partial,
            },
            Err(QualificationValidationError::Structural { reason }) => {
                Self::Invalid { reason }
            }
        }
    }
}

impl<T> QualificationDecision<T> {
    pub fn is_valid(&self) -> bool {
        matches!(self, Self::Valid(_))
    }

    pub fn is_unresolved(&self) -> bool {
        matches!(self, Self::Unresolved { .. })
    }

    pub fn missing(&self) -> &[IdentityRef] {
        match self {
            Self::Unresolved { missing, .. } => missing,
            Self::Valid(_) | Self::Invalid { .. } => &[],
        }
    }

    pub fn invalid_reason(&self) -> Option<&str> {
        match self {
            Self::Invalid { reason } => Some(reason),
            Self::Valid(_) | Self::Unresolved { .. } => None,
        }
    }
}

fn canonicalize_missing(mut missing: Vec<IdentityRef>) -> Vec<IdentityRef> {
    missing.sort_by(|left, right| {
        (&left.namespace, &left.kind, &left.id)
            .cmp(&(&right.namespace, &right.kind, &right.id))
    });
    missing.dedup();
    missing
}

impl<T> QualificationStatus<T> {
    pub fn unresolved(missing: Vec<IdentityRef>, partial: T) -> Self {
        Self::Unresolved {
            missing: canonicalize_missing(missing),
            partial,
        }
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

    /// Combines two independent qualification paths without collapsing an
    /// unresolved dependency state.
    ///
    /// Both partial qualification values are retained even when either path
    /// is unresolved. Missing dependency identities are unioned and
    /// canonicalized, making this a deterministic product operation for
    /// independently qualified evidence paths.
    pub fn zip<U>(self, other: QualificationStatus<U>) -> QualificationStatus<(T, U)> {
        match (self, other) {
            (Self::Complete(left), Self::Complete(right)) => {
                QualificationStatus::Complete((left, right))
            }
            (
                Self::Unresolved {
                    missing,
                    partial: left,
                },
                Self::Complete(right),
            )
            | (
                Self::Complete(left),
                Self::Unresolved {
                    missing,
                    partial: right,
                },
            ) => QualificationStatus::Unresolved {
                missing: canonicalize_missing(missing),
                partial: (left, right),
            },
            (
                Self::Unresolved {
                    mut missing,
                    partial: left,
                },
                Self::Unresolved {
                    missing: right_missing,
                    partial: right,
                },
            ) => {
                missing.extend(right_missing);
                QualificationStatus::Unresolved {
                    missing: canonicalize_missing(missing),
                    partial: (left, right),
                }
            }
        }
    }

    pub fn normalize_missing(&mut self) {
        if let Self::Unresolved { missing, .. } = self {
            *missing = canonicalize_missing(std::mem::take(missing));
        }
    }

    pub fn map<U>(self, map: impl FnOnce(T) -> U) -> QualificationStatus<U> {
        match self {
            Self::Complete(value) => QualificationStatus::Complete(map(value)),
            Self::Unresolved { missing, partial } => {
                QualificationStatus::unresolved(missing, map(partial))
            }
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
    fn outcome_maps_to_valid_decision() {
        let decision = QualificationDecision::from(
            Ok::<_, QualificationValidationError>(QualificationStatus::Complete(7u8)),
        );

        assert_eq!(decision, QualificationDecision::Valid(7u8));
        assert!(decision.is_valid());
        assert!(!decision.is_unresolved());
        assert_eq!(decision.missing(), &[]);
    }

    #[test]
    fn outcome_maps_to_unresolved_decision_and_canonicalizes_dependencies() {
        let decision = QualificationDecision::from(Ok::<_, QualificationValidationError>(
            QualificationStatus::Unresolved {
                missing: vec![id("z"), id("a"), id("z")],
                partial: 7u8,
            },
        ));

        assert_eq!(
            decision,
            QualificationDecision::Unresolved {
                missing: vec![id("a"), id("z")],
                partial: 7u8,
            }
        );
        assert!(decision.is_unresolved());
        assert_eq!(decision.missing(), &[id("a"), id("z")]);
    }

    #[test]
    fn outcome_maps_structural_error_to_invalid_decision() {
        let decision = QualificationDecision::from(
            Err::<QualificationStatus<u8>, _>(
                QualificationValidationError::structural("contradiction"),
            ),
        );

        assert_eq!(
            decision,
            QualificationDecision::Invalid {
                reason: "contradiction".into(),
            }
        );
        assert_eq!(decision.invalid_reason(), Some("contradiction"));
        assert!(!decision.is_valid());
        assert!(!decision.is_unresolved());
    }

    #[test]
    fn decision_projection_has_mutually_exclusive_terminal_states() {
        let valid = QualificationDecision::Valid(1u8);
        let invalid = QualificationDecision::Invalid {
            reason: "x".into(),
        };
        let unresolved = QualificationDecision::Unresolved {
            missing: vec![id("m")],
            partial: 1u8,
        };

        assert!(valid.is_valid());
        assert!(!valid.is_unresolved());
        assert_eq!(valid.invalid_reason(), None);
        assert!(!invalid.is_valid());
        assert!(!invalid.is_unresolved());
        assert_eq!(invalid.missing(), &[]);
        assert!(!unresolved.is_valid());
        assert!(unresolved.is_unresolved());
        assert_eq!(unresolved.invalid_reason(), None);
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
    fn unresolved_orders_missing_by_namespace_kind_and_id() {
        let mut first = id("z");
        first.namespace = "zeta".into();
        first.kind = IdentityKind::EvidenceRecord;

        let mut second = id("a");
        second.namespace = "alpha".into();
        second.kind = IdentityKind::ReconciliationWitness;

        let mut third = id("b");
        third.namespace = "alpha".into();
        third.kind = IdentityKind::EvidenceRecord;

        let status = QualificationStatus::unresolved(
            vec![first.clone(), second.clone(), third.clone()],
            7u8,
        );

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![third, second, first],
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
    fn zip_preserves_both_complete_partials() {
        let status = QualificationStatus::Complete(1u8).zip(QualificationStatus::Complete(2u8));
        assert_eq!(status, QualificationStatus::Complete((1u8, 2u8)));
    }

    #[test]
    fn zip_accumulates_one_unresolved_dependency_set() {
        let status = QualificationStatus::Complete(1u8).zip(QualificationStatus::Unresolved {
            missing: vec![id("b"), id("a"), id("b")],
            partial: 2u8,
        });

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![id("a"), id("b")],
                partial: (1u8, 2u8),
            }
        );
    }

    #[test]
    fn zip_accumulates_unresolved_dependency_when_it_is_the_left_path() {
        let status = QualificationStatus::Unresolved {
            missing: vec![id("left")],
            partial: 1u8,
        }
        .zip(QualificationStatus::Complete(2u8));

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![id("left")],
                partial: (1u8, 2u8),
            }
        );
    }

    #[test]
    fn zip_unions_two_unresolved_dependency_sets() {
        let status = QualificationStatus::Unresolved {
            missing: vec![id("z"), id("a")],
            partial: 1u8,
        }
        .zip(QualificationStatus::Unresolved {
            missing: vec![id("m"), id("a")],
            partial: 2u8,
        });

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![id("a"), id("m"), id("z")],
                partial: (1u8, 2u8),
            }
        );
    }

    #[test]
    fn zip_is_order_independent_for_missing_dependencies() {
        let left = QualificationStatus::Unresolved {
            missing: vec![id("b"), id("a")],
            partial: 1u8,
        }
        .zip(QualificationStatus::Unresolved {
            missing: vec![id("c")],
            partial: 2u8,
        });

        let right = QualificationStatus::Unresolved {
            missing: vec![id("c")],
            partial: 2u8,
        }
        .zip(QualificationStatus::Unresolved {
            missing: vec![id("a"), id("b")],
            partial: 1u8,
        });

        assert_eq!(left.missing(), right.missing());
    }

    #[test]
    fn zip_preserves_partial_payloads_even_when_unresolved() {
        let status = QualificationStatus::Unresolved {
            missing: vec![id("left-missing")],
            partial: 11u8,
        }
        .zip(QualificationStatus::Unresolved {
            missing: vec![id("right-missing")],
            partial: 22u8,
        });

        assert_eq!(status.missing().len(), 2);
        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![id("left-missing"), id("right-missing")],
                partial: (11u8, 22u8),
            }
        );
    }

    #[test]
    fn require_missing_is_associative_over_dependency_sets() {
        let a = id("a");
        let b = id("b");
        let c = id("c");

        let grouped = QualificationStatus::Complete(())
            .require_missing(vec![a.clone()])
            .require_missing(vec![b.clone()])
            .require_missing(vec![c.clone()]);
        let batched = QualificationStatus::Complete(())
            .require_missing(vec![c, a, b]);

        assert_eq!(grouped.missing(), batched.missing());
    }

    #[test]
    fn zip_has_grouping_invariant_missing_dependencies() {
        let first = QualificationStatus::Unresolved {
            missing: vec![id("a")],
            partial: 1u8,
        };
        let second = QualificationStatus::Unresolved {
            missing: vec![id("b")],
            partial: 2u8,
        };
        let third = QualificationStatus::Unresolved {
            missing: vec![id("c")],
            partial: 3u8,
        };

        let left_grouped = first.clone().zip(second.clone()).zip(third.clone());
        let right_grouped = first.zip(second.zip(third));

        assert_eq!(left_grouped.missing(), right_grouped.missing());
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
    fn map_recanonicalizes_unresolved_dependencies() {
        let status = QualificationStatus::Unresolved {
            missing: vec![id("z"), id("a"), id("z")],
            partial: 3u8,
        }
        .map(|value| value * 2);

        assert_eq!(
            status,
            QualificationStatus::Unresolved {
                missing: vec![id("a"), id("z")],
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

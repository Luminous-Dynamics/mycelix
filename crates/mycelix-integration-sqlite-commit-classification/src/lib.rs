//! Provider-neutral classification of exact durable commit outcomes.
//!
//! This crate does not execute SQLite transactions and does not infer durability
//! from error strings, connection-local state, or source ordering. Callers must
//! independently re-read the exact durable predecessor/successor identity and
//! supply only those positive exact-state observations here.

pub const SQLITE_COMMIT_CLASSIFICATION_PROFILE: &str =
    "mycelix-integration-sqlite-commit-classification-v1";

/// Durable semantic classification after a critical transaction attempt.
///
/// `DefinitelyNotCommitted` means the complete exact predecessor was positively
/// re-observed. It does not itself authorize retry; the enclosing authority,
/// lease, attempt, and currentness policy must still permit another operation.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CommitClassification {
    Committed,
    DefinitelyNotCommitted,
    IndeterminateCommit,
}

impl CommitClassification {
    pub const fn successor_proven_here(self) -> bool {
        matches!(self, Self::Committed)
    }

    pub const fn exact_predecessor_proven_here(self) -> bool {
        matches!(self, Self::DefinitelyNotCommitted)
    }

    pub const fn recovery_only_here(self) -> bool {
        matches!(self, Self::IndeterminateCommit)
    }

    /// Persistence classification is never execution authority.
    pub const fn grants_execution_authority(self) -> bool {
        false
    }

    /// Even a proven predecessor only makes retry *consideration* possible.
    /// It does not itself authorize a retry.
    pub const fn authorizes_retry(self) -> bool {
        false
    }
}

/// Classify only from positive exact durable observations.
///
/// Any contradictory observation (`successor_exact && predecessor_exact`) or
/// inability to prove either exact state is conservatively indeterminate.
pub const fn classify_exact_durable_transition(
    successor_exact: bool,
    predecessor_exact: bool,
) -> CommitClassification {
    match (successor_exact, predecessor_exact) {
        (true, false) => CommitClassification::Committed,
        (false, true) => CommitClassification::DefinitelyNotCommitted,
        _ => CommitClassification::IndeterminateCommit,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_successor_exact_is_committed() {
        assert_eq!(
            classify_exact_durable_transition(true, false),
            CommitClassification::Committed
        );
    }

    #[test]
    fn only_predecessor_exact_is_definitely_not_committed() {
        assert_eq!(
            classify_exact_durable_transition(false, true),
            CommitClassification::DefinitelyNotCommitted
        );
    }

    #[test]
    fn unknown_or_conflicting_state_is_indeterminate() {
        assert_eq!(
            classify_exact_durable_transition(false, false),
            CommitClassification::IndeterminateCommit
        );
        assert_eq!(
            classify_exact_durable_transition(true, true),
            CommitClassification::IndeterminateCommit
        );
    }

    #[test]
    fn classification_never_grants_authority_or_retry() {
        for classification in [
            CommitClassification::Committed,
            CommitClassification::DefinitelyNotCommitted,
            CommitClassification::IndeterminateCommit,
        ] {
            assert!(!classification.grants_execution_authority());
            assert!(!classification.authorizes_retry());
        }
    }
}

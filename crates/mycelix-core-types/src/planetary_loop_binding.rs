// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
// Commercial licensing: see COMMERCIAL_LICENSE.md at repository root
//! Cross-artifact binding helpers for the response lifecycle.
//!
//! These checks are intentionally semantic and non-authoritative. They prove
//! that separately validated artifacts refer to the same response root; they do
//! not prove signatures, actor authority, or execution permission.

use crate::{
    ExecutionReceipt, ResponseArtifactKind, ResponseArtifactRef, ResponseLifecycleError,
    ResponseLifecycleLedger, ResponseProposal,
};

/// Verify that a lifecycle ledger and an execution receipt are rooted in the
/// same response proposal and selected option.
pub fn validate_lifecycle_execution_binding(
    ledger: &ResponseLifecycleLedger,
    proposal: &ResponseProposal,
    execution: &ExecutionReceipt,
) -> Result<(), LifecycleBindingError> {
    ledger.validate().map_err(LifecycleBindingError::Lifecycle)?;
    proposal.validate().map_err(|error| LifecycleBindingError::Proposal(error.to_string()))?;
    execution
        .validate_against_proposal(proposal)
        .map_err(|error| LifecycleBindingError::Execution(error.to_string()))?;

    if ledger.proposal.kind != ResponseArtifactKind::Proposal
        || ledger.proposal.id != proposal.id
    {
        return Err(LifecycleBindingError::ProposalRootMismatch);
    }

    let Some(bound_execution) = ledger.execution.as_ref() else {
        return Err(LifecycleBindingError::ExecutionNotBound);
    };
    if bound_execution.kind != ResponseArtifactKind::ExecutionReceipt
        || bound_execution.id != execution.id
    {
        return Err(LifecycleBindingError::ExecutionMismatch);
    }

    Ok(())
}

/// Check whether a candidate artifact reference is exactly the one bound in a
/// lifecycle slot. Digest equality is required; matching IDs alone are not
/// sufficient because content-addressed identity is the tamper boundary.
pub fn artifact_ref_matches(
    expected: &ResponseArtifactRef,
    candidate: &ResponseArtifactRef,
) -> bool {
    expected == candidate
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum LifecycleBindingError {
    Lifecycle(ResponseLifecycleError),
    Proposal(String),
    Execution(String),
    ProposalRootMismatch,
    ExecutionNotBound,
    ExecutionMismatch,
}

impl core::fmt::Display for LifecycleBindingError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Lifecycle(error) => write!(f, "invalid lifecycle: {error}"),
            Self::Proposal(error) => write!(f, "invalid response proposal: {error}"),
            Self::Execution(error) => write!(f, "invalid execution receipt: {error}"),
            Self::ProposalRootMismatch => {
                write!(f, "lifecycle proposal root does not match response proposal")
            }
            Self::ExecutionNotBound => write!(f, "lifecycle does not bind an execution receipt"),
            Self::ExecutionMismatch => {
                write!(f, "execution receipt does not match lifecycle-bound execution")
            }
        }
    }
}

impl std::error::Error for LifecycleBindingError {}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn artifact_reference_requires_digest_equality() {
        let left = ResponseArtifactRef::new(
            ResponseArtifactKind::ExecutionReceipt,
            "execution:1",
            "sha256:one",
        )
        .unwrap();
        let right = ResponseArtifactRef::new(
            ResponseArtifactKind::ExecutionReceipt,
            "execution:1",
            "sha256:two",
        )
        .unwrap();
        assert!(!artifact_ref_matches(&left, &right));
    }
}

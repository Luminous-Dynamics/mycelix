// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Institutional `Digest32` adapter for the shared exact-byte execution-action core.
//!
//! The framing rule lives in `mycelix-execution-action-digest-core`. This crate
//! preserves the governance-facing `Digest32` API without owning a second BLAKE3
//! implementation.

use mycelix_execution_action_digest_core::execution_authority_digest_bytes;
use mycelix_institutional_core::Digest32;

pub use mycelix_execution_action_digest_core::{
    ACTIONS_DIGEST_PROFILE_V1, ActionDigestError, MAX_ACTION_BYTES, MAX_PROPOSAL_ID_BYTES,
};

/// Digest one exact proposal/action byte pair using the registered governance
/// execution profile and project the shared raw digest into institutional `Digest32`.
pub fn execution_authority_digest(
    proposal_id: &str,
    actions: &str,
) -> Result<Digest32, ActionDigestError> {
    execution_authority_digest_bytes(proposal_id, actions).map(Digest32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn institutional_adapter_is_exact_core_projection() {
        let proposal = "response:1";
        let actions = "[{\"type\":\"noop\"}]";
        let adapted = execution_authority_digest(proposal, actions).unwrap();
        let raw = execution_authority_digest_bytes(proposal, actions).unwrap();
        assert_eq!(adapted, Digest32(raw));
    }

    #[test]
    fn exact_bytes_are_stable_and_whitespace_sensitive() {
        let exact = "[{\"type\":\"noop\"}]";
        let spaced = "[{ \"type\": \"noop\" }]";
        let a = execution_authority_digest("response:1", exact).unwrap();
        let b = execution_authority_digest("response:1", exact).unwrap();
        let c = execution_authority_digest("response:1", spaced).unwrap();
        assert_eq!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn proposal_identity_is_part_of_action_identity() {
        let actions = "[{\"type\":\"noop\"}]";
        let a = execution_authority_digest("response:1", actions).unwrap();
        let b = execution_authority_digest("response:2", actions).unwrap();
        assert_ne!(a, b);
    }

    #[test]
    fn action_order_is_exact_byte_sensitive() {
        let a = execution_authority_digest(
            "response:1",
            "[{\"type\":\"a\"},{\"type\":\"b\"}]",
        )
        .unwrap();
        let b = execution_authority_digest(
            "response:1",
            "[{\"type\":\"b\"},{\"type\":\"a\"}]",
        )
        .unwrap();
        assert_ne!(a, b);
    }

    #[test]
    fn empty_and_oversized_inputs_fail_closed() {
        assert_eq!(
            execution_authority_digest("", "[]").unwrap_err(),
            ActionDigestError::EmptyProposalId
        );
        assert_eq!(
            execution_authority_digest("response:1", "").unwrap_err(),
            ActionDigestError::EmptyActions
        );
        let oversized = "x".repeat(MAX_ACTION_BYTES + 1);
        assert!(matches!(
            execution_authority_digest("response:1", &oversized),
            Err(ActionDigestError::ActionsTooLong { .. })
        ));
    }
}

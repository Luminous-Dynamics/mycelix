// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Shared exact-byte digest for governance execution actions.
//!
//! This crate extracts the byte rule historically used by the governance
//! execution-plan verifier so response and governance layers cannot silently
//! drift into different action identities.
//!
//! The digest is deliberately **not** JSON canonicalization. Callers that require
//! JSON must validate it separately, then pass the exact bytes they intend to
//! execute. Whitespace, key order, escaping, proposal identity, or any other byte
//! change must produce a different digest.

use mycelix_institutional_core::Digest32;
use std::fmt;

pub const ACTIONS_DIGEST_PROFILE_V1: &str =
    "mycelix-governance-execution-authority-v1-blake3-exact-json";
pub const MAX_ACTION_BYTES: usize = 4096;
pub const MAX_PROPOSAL_ID_BYTES: usize = 512;

const EXECUTION_AUTHORITY_DOMAIN: &[u8] = b"mycelix-governance-execution-authority-v1\0";

/// Digest one exact proposal/action byte pair using the registered governance
/// execution profile.
///
/// This function intentionally does not trim, parse, reorder, or normalize
/// `actions`. A separate caller-side contract may require valid JSON, but the
/// digest itself always commits the exact UTF-8 bytes supplied here.
pub fn execution_authority_digest(
    proposal_id: &str,
    actions: &str,
) -> Result<Digest32, ActionDigestError> {
    if proposal_id.trim().is_empty() {
        return Err(ActionDigestError::EmptyProposalId);
    }
    if proposal_id.len() > MAX_PROPOSAL_ID_BYTES {
        return Err(ActionDigestError::ProposalIdTooLong {
            actual: proposal_id.len(),
            max: MAX_PROPOSAL_ID_BYTES,
        });
    }
    if actions.is_empty() {
        return Err(ActionDigestError::EmptyActions);
    }
    if actions.len() > MAX_ACTION_BYTES {
        return Err(ActionDigestError::ActionsTooLong {
            actual: actions.len(),
            max: MAX_ACTION_BYTES,
        });
    }

    let mut hasher = blake3::Hasher::new();
    hasher.update(EXECUTION_AUTHORITY_DOMAIN);
    hasher.update(&(proposal_id.len() as u64).to_le_bytes());
    hasher.update(proposal_id.as_bytes());
    hasher.update(&(actions.len() as u64).to_le_bytes());
    hasher.update(actions.as_bytes());
    Ok(Digest32(*hasher.finalize().as_bytes()))
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ActionDigestError {
    EmptyProposalId,
    ProposalIdTooLong { actual: usize, max: usize },
    EmptyActions,
    ActionsTooLong { actual: usize, max: usize },
}

impl fmt::Display for ActionDigestError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::EmptyProposalId => write!(f, "execution digest proposal id must not be empty"),
            Self::ProposalIdTooLong { actual, max } => write!(
                f,
                "execution digest proposal id is {actual} bytes; maximum is {max}"
            ),
            Self::EmptyActions => write!(f, "execution action bytes must not be empty"),
            Self::ActionsTooLong { actual, max } => write!(
                f,
                "execution action payload is {actual} bytes; maximum is {max}"
            ),
        }
    }
}

impl std::error::Error for ActionDigestError {}

#[cfg(test)]
mod tests {
    use super::*;

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

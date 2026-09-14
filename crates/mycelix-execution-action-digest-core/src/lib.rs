// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Substrate-neutral exact-byte digest for governance execution actions.
//!
//! This is the narrow protocol waist beneath institutional `Digest32` wrappers,
//! Commons hex-string evidence, response intent, and other execution consumers.
//! It owns only the registered byte framing. It does not own authority semantics.
//!
//! The digest is deliberately **not** JSON canonicalization. Callers that require
//! JSON must validate it separately and then pass the exact UTF-8 bytes they mean
//! to authorize. Whitespace, key order, escaping, action order, and proposal
//! identity are digest-significant.

use std::fmt;

pub const ACTIONS_DIGEST_PROFILE_V1: &str =
    "mycelix-governance-execution-authority-v1-blake3-exact-json";
pub const MAX_ACTION_BYTES: usize = 4096;
pub const MAX_PROPOSAL_ID_BYTES: usize = 512;

const EXECUTION_AUTHORITY_DOMAIN: &[u8] = b"mycelix-governance-execution-authority-v1\0";

/// Compute the registered v1 governance execution digest as raw bytes.
pub fn execution_authority_digest_bytes(
    proposal_id: &str,
    actions: &str,
) -> Result<[u8; 32], ActionDigestError> {
    validate_inputs(proposal_id, actions)?;
    let mut hasher = blake3::Hasher::new();
    hasher.update(EXECUTION_AUTHORITY_DOMAIN);
    hasher.update(&(proposal_id.len() as u64).to_le_bytes());
    hasher.update(proposal_id.as_bytes());
    hasher.update(&(actions.len() as u64).to_le_bytes());
    hasher.update(actions.as_bytes());
    Ok(*hasher.finalize().as_bytes())
}

/// Compute the same registered identity as lowercase 64-hex for evidence layers
/// that intentionally do not depend on an institutional digest wrapper type.
pub fn execution_authority_digest_hex(
    proposal_id: &str,
    actions: &str,
) -> Result<String, ActionDigestError> {
    Ok(hex_32(execution_authority_digest_bytes(
        proposal_id,
        actions,
    )?))
}

fn validate_inputs(proposal_id: &str, actions: &str) -> Result<(), ActionDigestError> {
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
    Ok(())
}

fn hex_32(bytes: [u8; 32]) -> String {
    const HEX: &[u8; 16] = b"0123456789abcdef";
    let mut output = String::with_capacity(64);
    for byte in bytes {
        output.push(HEX[(byte >> 4) as usize] as char);
        output.push(HEX[(byte & 0x0f) as usize] as char);
    }
    output
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
    fn byte_and_hex_views_are_one_identity() {
        let bytes = execution_authority_digest_bytes("MIP-42", "[{\"type\":\"noop\"}]").unwrap();
        let hex = execution_authority_digest_hex("MIP-42", "[{\"type\":\"noop\"}]").unwrap();
        assert_eq!(hex, hex_32(bytes));
    }

    #[test]
    fn exact_bytes_are_stable_and_whitespace_sensitive() {
        let exact = "[{\"type\":\"noop\"}]";
        let spaced = "[{ \"type\": \"noop\" }]";
        let a = execution_authority_digest_bytes("MIP-42", exact).unwrap();
        let b = execution_authority_digest_bytes("MIP-42", exact).unwrap();
        let c = execution_authority_digest_bytes("MIP-42", spaced).unwrap();
        assert_eq!(a, b);
        assert_ne!(a, c);
    }

    #[test]
    fn proposal_identity_is_digest_significant() {
        let actions = "[{\"type\":\"noop\"}]";
        let a = execution_authority_digest_bytes("MIP-42", actions).unwrap();
        let b = execution_authority_digest_bytes("MIP-43", actions).unwrap();
        assert_ne!(a, b);
    }

    #[test]
    fn action_order_is_exact_byte_sensitive() {
        let a = execution_authority_digest_bytes("MIP-42", "[{\"type\":\"a\"},{\"type\":\"b\"}]")
            .unwrap();
        let b = execution_authority_digest_bytes("MIP-42", "[{\"type\":\"b\"},{\"type\":\"a\"}]")
            .unwrap();
        assert_ne!(a, b);
    }

    #[test]
    fn empty_and_oversized_inputs_fail_closed() {
        assert_eq!(
            execution_authority_digest_bytes("", "[]").unwrap_err(),
            ActionDigestError::EmptyProposalId
        );
        assert_eq!(
            execution_authority_digest_bytes("MIP-42", "").unwrap_err(),
            ActionDigestError::EmptyActions
        );
        let oversized = "x".repeat(MAX_ACTION_BYTES + 1);
        assert!(matches!(
            execution_authority_digest_bytes("MIP-42", &oversized),
            Err(ActionDigestError::ActionsTooLong { .. })
        ));
    }
}

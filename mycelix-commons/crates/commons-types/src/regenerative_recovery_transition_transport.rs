// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Self-contained transport integrity for recovery transition commitments.
//!
//! This layer is deliberately weaker than exact transition recomputation. It can
//! validate that deserialized bytes are internally coherent and content-addressed,
//! but it cannot prove governance authority, source-object provenance, currentness,
//! or persistence authority. A trusted runtime must still re-run the stronger
//! transition theorem against authoritative live inputs before any mutation.

use crate::{
    REGENERATIVE_RECOVERY_TRANSITION_COMMITMENT_SCHEMA_V1,
    RegenerativeRecoveryForkResolutionOutcomeV1, RegenerativeRecoveryTransitionCommitmentV1,
};
use serde::{Deserialize, Serialize};

pub const REGENERATIVE_RECOVERY_TRANSITION_TRANSPORT_SCHEMA_V1: u8 = 1;

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeRecoveryTransitionTransportV1 {
    pub schema_version: u8,
    pub commitment: RegenerativeRecoveryTransitionCommitmentV1,
    pub commitment_content_digest: String,
}

impl RegenerativeRecoveryTransitionTransportV1 {
    /// Build a transport envelope from an in-memory commitment.
    ///
    /// This establishes only internal transport integrity. It does not upgrade the
    /// commitment into governance or persistence authority.
    pub fn from_commitment(
        commitment: RegenerativeRecoveryTransitionCommitmentV1,
    ) -> Result<Self, String> {
        validate_commitment_structure(&commitment)?;
        let commitment_content_digest = commitment.content_digest()?;
        let transport = Self {
            schema_version: REGENERATIVE_RECOVERY_TRANSITION_TRANSPORT_SCHEMA_V1,
            commitment,
            commitment_content_digest,
        };
        transport.validate()?;
        Ok(transport)
    }

    /// Validate only self-contained transport invariants.
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_RECOVERY_TRANSITION_TRANSPORT_SCHEMA_V1 {
            return Err("unsupported recovery transition transport schema".into());
        }
        validate_commitment_structure(&self.commitment)?;
        validate_digest(
            "commitment_content_digest",
            &self.commitment_content_digest,
        )?;
        let recomputed = self.commitment.content_digest()?;
        if recomputed != self.commitment_content_digest {
            return Err("recovery transition transport content digest mismatch".into());
        }
        Ok(())
    }

    /// Transport integrity never proves that the original transition inputs were
    /// authoritative, current, or even available to this verifier.
    pub const fn exact_source_recomputation_verified_here(&self) -> bool {
        false
    }

    pub const fn governance_authority_verified_here(&self) -> bool {
        false
    }

    pub const fn persistence_authorized_here(&self) -> bool {
        false
    }
}

pub fn validate_regenerative_recovery_transition_transport(
    transport: &RegenerativeRecoveryTransitionTransportV1,
) -> Result<(), String> {
    transport.validate()
}

fn validate_commitment_structure(
    commitment: &RegenerativeRecoveryTransitionCommitmentV1,
) -> Result<(), String> {
    if commitment.schema_version != REGENERATIVE_RECOVERY_TRANSITION_COMMITMENT_SCHEMA_V1 {
        return Err("unsupported recovery transition commitment schema".into());
    }

    for (name, digest) in [
        ("pre_state_digest", commitment.pre_state_digest.as_str()),
        (
            "fork_resolution_content_digest",
            commitment.fork_resolution_content_digest.as_str(),
        ),
        (
            "current_branch_content_digest",
            commitment.current_branch_content_digest.as_str(),
        ),
        (
            "sibling_branch_content_digest",
            commitment.sibling_branch_content_digest.as_str(),
        ),
        ("post_state_digest", commitment.post_state_digest.as_str()),
    ] {
        validate_digest(name, digest)?;
    }

    if commitment.current_branch_content_digest == commitment.sibling_branch_content_digest {
        return Err("recovery transition branches must have distinct content digests".into());
    }

    if let Some(selected) = commitment.selected_branch_content_digest.as_deref() {
        validate_digest("selected_branch_content_digest", selected)?;
    }

    match commitment.outcome {
        RegenerativeRecoveryForkResolutionOutcomeV1::SuspendReserve => {
            if commitment.selected_branch_content_digest.is_some() {
                return Err("suspended recovery transition must not select a branch".into());
            }
            if commitment.cursor_resumed {
                return Err("suspended recovery transition cannot report a resumed cursor".into());
            }
            if commitment.pre_state_digest != commitment.post_state_digest {
                return Err("suspended recovery transition must preserve exact cursor state".into());
            }
        }
        RegenerativeRecoveryForkResolutionOutcomeV1::RetainSelectedBranch => {
            let selected = commitment
                .selected_branch_content_digest
                .as_deref()
                .ok_or_else(|| "retained recovery transition must select a branch".to_string())?;
            if selected != commitment.current_branch_content_digest
                && selected != commitment.sibling_branch_content_digest
            {
                return Err(
                    "selected recovery transition branch is not one of the committed fork branches"
                        .into(),
                );
            }
            if !commitment.cursor_resumed {
                return Err("retained recovery transition must report a resumed cursor".into());
            }
            if commitment.pre_state_digest == commitment.post_state_digest {
                return Err("retained recovery transition must clear or change frozen cursor state".into());
            }
        }
    }

    Ok(())
}

fn validate_digest(name: &str, digest: &str) -> Result<(), String> {
    if digest.len() != 64
        || !digest
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
    {
        return Err(format!(
            "{name} must be exactly 64 lowercase hexadecimal characters"
        ));
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn digest(byte: &str) -> String {
        byte.repeat(64)
    }

    fn retained_commitment() -> RegenerativeRecoveryTransitionCommitmentV1 {
        RegenerativeRecoveryTransitionCommitmentV1 {
            schema_version: REGENERATIVE_RECOVERY_TRANSITION_COMMITMENT_SCHEMA_V1,
            pre_state_digest: digest("a"),
            fork_resolution_content_digest: digest("b"),
            current_branch_content_digest: digest("c"),
            sibling_branch_content_digest: digest("d"),
            outcome: RegenerativeRecoveryForkResolutionOutcomeV1::RetainSelectedBranch,
            selected_branch_content_digest: Some(digest("d")),
            post_state_digest: digest("e"),
            cursor_resumed: true,
        }
    }

    fn suspended_commitment() -> RegenerativeRecoveryTransitionCommitmentV1 {
        RegenerativeRecoveryTransitionCommitmentV1 {
            schema_version: REGENERATIVE_RECOVERY_TRANSITION_COMMITMENT_SCHEMA_V1,
            pre_state_digest: digest("a"),
            fork_resolution_content_digest: digest("b"),
            current_branch_content_digest: digest("c"),
            sibling_branch_content_digest: digest("d"),
            outcome: RegenerativeRecoveryForkResolutionOutcomeV1::SuspendReserve,
            selected_branch_content_digest: None,
            post_state_digest: digest("a"),
            cursor_resumed: false,
        }
    }

    #[test]
    fn retained_transport_round_trips_without_gaining_authority() {
        let transport = RegenerativeRecoveryTransitionTransportV1::from_commitment(
            retained_commitment(),
        )
        .unwrap();
        transport.validate().unwrap();
        let bytes = serde_json::to_vec(&transport).unwrap();
        let decoded: RegenerativeRecoveryTransitionTransportV1 =
            serde_json::from_slice(&bytes).unwrap();
        decoded.validate().unwrap();
        assert_eq!(transport, decoded);
        assert!(!decoded.exact_source_recomputation_verified_here());
        assert!(!decoded.governance_authority_verified_here());
        assert!(!decoded.persistence_authorized_here());
    }

    #[test]
    fn suspended_transport_requires_exact_no_state_change_semantics() {
        let mut commitment = suspended_commitment();
        RegenerativeRecoveryTransitionTransportV1::from_commitment(commitment.clone()).unwrap();

        commitment.post_state_digest = digest("e");
        assert!(RegenerativeRecoveryTransitionTransportV1::from_commitment(commitment).is_err());
    }

    #[test]
    fn retained_selection_must_name_one_committed_fork_branch() {
        let mut commitment = retained_commitment();
        commitment.selected_branch_content_digest = Some(digest("f"));
        assert!(RegenerativeRecoveryTransitionTransportV1::from_commitment(commitment).is_err());
    }

    #[test]
    fn contradictory_cursor_flags_fail_closed() {
        let mut suspended = suspended_commitment();
        suspended.cursor_resumed = true;
        assert!(RegenerativeRecoveryTransitionTransportV1::from_commitment(suspended).is_err());

        let mut retained = retained_commitment();
        retained.cursor_resumed = false;
        assert!(RegenerativeRecoveryTransitionTransportV1::from_commitment(retained).is_err());
    }

    #[test]
    fn malformed_or_noncanonical_digest_fields_fail_closed() {
        let mut commitment = retained_commitment();
        commitment.pre_state_digest = "AA".repeat(32);
        assert!(RegenerativeRecoveryTransitionTransportV1::from_commitment(commitment).is_err());
    }

    #[test]
    fn envelope_digest_detects_transport_tampering() {
        let mut transport = RegenerativeRecoveryTransitionTransportV1::from_commitment(
            retained_commitment(),
        )
        .unwrap();
        transport.commitment.post_state_digest = digest("f");
        assert!(validate_regenerative_recovery_transition_transport(&transport).is_err());
    }

    #[test]
    fn duplicate_branch_identity_is_rejected() {
        let mut commitment = retained_commitment();
        commitment.sibling_branch_content_digest = commitment.current_branch_content_digest.clone();
        commitment.selected_branch_content_digest =
            Some(commitment.current_branch_content_digest.clone());
        assert!(RegenerativeRecoveryTransitionTransportV1::from_commitment(commitment).is_err());
    }
}

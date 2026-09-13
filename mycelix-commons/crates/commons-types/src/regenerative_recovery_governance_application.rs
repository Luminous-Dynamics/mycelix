// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Semantic composition between one recovery-reserve fork decision and Mycelix's
//! stable threshold-authorization identity.
//!
//! Commons can prove that caller-supplied evidence is internally bound to the
//! exact fork resolution, exact governance action bytes, and one stable threshold
//! authorization identity. Commons does **not** verify the upstream institutional
//! policy, signatures, committee/key provenance, revocation state, or current
//! provider authority.
//!
//! That distinction is authority-significant: an internally consistent portable
//! evidence record is not permission to mutate the recovery cursor. The actual
//! #760 cursor mutation must therefore occur only in a trusted provider/runtime
//! layer after #69/#71/#82 authority has been independently verified.

use crate::{
    RegenerativeRecoveryCoordinateEvidenceV1, RegenerativeRecoveryForkResolutionEvidenceV1,
    RegenerativeRecoveryGovernanceAuthorityEvidenceV1, RegenerativeRecoveryReserveHeadV1,
    verify_regenerative_recovery_governance_authority,
};

/// Portable proof that one exact recovery-fork resolution is semantically joined
/// to one exact governance action and stable threshold-authorization identity.
///
/// This receipt is deliberately **non-authorizing**. It carries no mutation
/// capability and permanently records that upstream authority was not verified
/// inside Commons.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RegenerativeRecoveryGovernanceBindingReceiptV1 {
    pub resolution_content_digest: String,
    pub governance_actions_digest: String,
    pub governance_actions_digest_profile: String,
    pub threshold_authorization_ref: String,
    pub threshold_authorization_identity_digest: String,
    pub threshold_authorization_identity_profile: String,
    pub threshold_qualification_evidence_binding: String,
    pub threshold_identity_evidence_binding: String,
}

impl RegenerativeRecoveryGovernanceBindingReceiptV1 {
    pub const fn upstream_threshold_authority_verified_here(&self) -> bool {
        false
    }

    pub const fn recovery_cursor_mutation_authorized_here(&self) -> bool {
        false
    }
}

/// Verify only the exact portable semantic join. No recovery state is mutated.
pub fn qualify_regenerative_recovery_fork_resolution_governance_binding(
    resolution: &RegenerativeRecoveryForkResolutionEvidenceV1,
    authority: &RegenerativeRecoveryGovernanceAuthorityEvidenceV1,
) -> Result<RegenerativeRecoveryGovernanceBindingReceiptV1, String> {
    let action = verify_regenerative_recovery_governance_authority(resolution, authority)?;
    let resolution_content_digest = resolution.content_digest()?;

    Ok(RegenerativeRecoveryGovernanceBindingReceiptV1 {
        resolution_content_digest,
        governance_actions_digest: action.actions_digest().to_string(),
        governance_actions_digest_profile: action.actions_digest_profile().to_string(),
        threshold_authorization_ref: authority.threshold_authorization_ref.clone(),
        threshold_authorization_identity_digest: authority
            .threshold_authorization_identity_digest
            .clone(),
        threshold_authorization_identity_profile: authority
            .threshold_authorization_identity_profile
            .clone(),
        threshold_qualification_evidence_binding: authority
            .threshold_qualification_evidence_binding
            .clone(),
        threshold_identity_evidence_binding: authority.threshold_identity_evidence_binding.clone(),
    })
}

/// Compatibility shape retained so downstream code does not silently reinterpret
/// an API disappearance as authority. No value of this type is minted by Commons
/// after the mutation-boundary hardening.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RegenerativeRecoveryGovernedResolutionReceiptV1 {
    pub resolution_content_digest: String,
    pub governance_actions_digest: String,
    pub governance_actions_digest_profile: String,
    pub threshold_authorization_identity_digest: String,
    pub threshold_authorization_identity_profile: String,
    pub cursor_resumed: bool,
}

impl RegenerativeRecoveryGovernedResolutionReceiptV1 {
    pub const fn upstream_threshold_authority_verified_here(&self) -> bool {
        false
    }

    pub const fn recovery_cursor_mutation_authorized_here(&self) -> bool {
        false
    }
}

/// Fail-closed compatibility shim for the former Commons mutation path.
///
/// The semantic binding is still checked first so malformed evidence is diagnosed
/// accurately, but this function never mutates `head`. A trusted provider must:
///
/// 1. independently verify #69/#71/#82 authority and current provider policy;
/// 2. bind that verified authority to the exact semantic receipt returned by
///    `qualify_regenerative_recovery_fork_resolution_governance_binding`; and only
/// 3. invoke the provider-neutral #760 mutation primitive inside that trusted
///    boundary.
pub fn apply_regenerative_recovery_fork_resolution_with_bound_governance_authority(
    _head: &mut RegenerativeRecoveryReserveHeadV1,
    _current_branch: &RegenerativeRecoveryCoordinateEvidenceV1,
    _sibling_branch: &RegenerativeRecoveryCoordinateEvidenceV1,
    resolution: &RegenerativeRecoveryForkResolutionEvidenceV1,
    authority: &RegenerativeRecoveryGovernanceAuthorityEvidenceV1,
) -> Result<RegenerativeRecoveryGovernedResolutionReceiptV1, String> {
    let _binding =
        qualify_regenerative_recovery_fork_resolution_governance_binding(resolution, authority)?;
    Err(
        "Commons does not verify upstream threshold authority; recovery cursor mutation requires a trusted provider after independent #69/#71/#82 verification"
            .into(),
    )
}

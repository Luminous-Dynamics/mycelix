// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Composed application path for a recovery-reserve fork decision that has been
//! bound to Mycelix's stable threshold-authorization identity.
//!
//! This wrapper intentionally sits above the provider-neutral #760 primitive. It
//! performs the exact governance semantic join first and mutates the frozen cursor
//! only after that join succeeds. Upstream threshold qualification / cryptographic
//! verification remain external evidence subjects; this module does not recreate
//! them.

use crate::{
    apply_regenerative_recovery_fork_resolution,
    verify_regenerative_recovery_governance_authority,
    RegenerativeRecoveryCoordinateEvidenceV1, RegenerativeRecoveryForkResolutionEvidenceV1,
    RegenerativeRecoveryGovernanceAuthorityEvidenceV1, RegenerativeRecoveryReserveHeadV1,
};

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
}

pub fn apply_regenerative_recovery_fork_resolution_with_bound_governance_authority(
    head: &mut RegenerativeRecoveryReserveHeadV1,
    current_branch: &RegenerativeRecoveryCoordinateEvidenceV1,
    sibling_branch: &RegenerativeRecoveryCoordinateEvidenceV1,
    resolution: &RegenerativeRecoveryForkResolutionEvidenceV1,
    authority: &RegenerativeRecoveryGovernanceAuthorityEvidenceV1,
) -> Result<RegenerativeRecoveryGovernedResolutionReceiptV1, String> {
    let action = verify_regenerative_recovery_governance_authority(resolution, authority)?;
    let resolution_content_digest = resolution.content_digest()?;

    let cursor_resumed = apply_regenerative_recovery_fork_resolution(
        head,
        current_branch,
        sibling_branch,
        resolution,
    )?;

    Ok(RegenerativeRecoveryGovernedResolutionReceiptV1 {
        resolution_content_digest,
        governance_actions_digest: action.actions_digest().to_string(),
        governance_actions_digest_profile: action.actions_digest_profile().to_string(),
        threshold_authorization_identity_digest: authority
            .threshold_authorization_identity_digest
            .clone(),
        threshold_authorization_identity_profile: authority
            .threshold_authorization_identity_profile
            .clone(),
        cursor_resumed,
    })
}

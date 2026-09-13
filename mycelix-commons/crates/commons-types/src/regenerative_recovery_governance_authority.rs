// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Portable governance-authority evidence for recovery-reserve fork resolution.
//!
//! This module deliberately does not depend on the Holochain governance zome
//! crates. Instead it preserves the minimum facts required to compose an exact
//! fork-resolution decision with an exact Mycelix governance proposal, threshold
//! signature, signing committee and independent signature-verification receipt.
//!
//! A `verified` boolean is intentionally not part of this evidence schema. The
//! threshold-signing integrity zome's structural validator accepts well-shaped
//! signatures independently of cryptographic verification, so recovery authority
//! must bind a separate verification evidence subject rather than self-asserting
//! finality.

use crate::{
    MaritimeEvidenceEnvelope, MaritimeEvidenceKind,
    RegenerativeRecoveryForkResolutionEvidenceV1,
};
use serde::{Deserialize, Serialize};

pub const REGENERATIVE_RECOVERY_GOVERNANCE_AUTHORITY_SCHEMA_V1: u8 = 1;
pub const RECOVERY_FORK_RESOLUTION_SCOPE_ID_V1: &str =
    "regenerative-recovery-fork-resolution";
const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 1024;

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RegenerativeRecoveryGovernanceProposalFinalityV1 {
    Signed,
    Executed,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RegenerativeRecoveryThresholdSignatureAlgorithmV1 {
    Ecdsa,
    MlDsa65,
    HybridEcdsaMlDsa65,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case", tag = "kind", content = "scope_ids")]
pub enum RegenerativeRecoveryGovernanceCommitteeScopeV1 {
    All,
    Constitutional,
    Treasury,
    Protocol,
    Custom(Vec<String>),
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeRecoveryGovernanceAuthorityEvidenceV1 {
    pub schema_version: u8,
    pub authority_evidence_id: String,
    /// Must equal the authority binding carried by the fork-resolution record.
    pub governance_authority_binding: String,
    /// Exact external binding for the fork-resolution record itself.
    pub resolution_evidence_binding: String,
    /// BLAKE3 content digest of the exact fork-resolution payload being authorized.
    pub resolution_content_digest: String,

    /// Exact Mycelix governance proposal subject.
    pub governance_proposal_id: String,
    pub governance_proposal_record_binding: String,
    pub governance_proposal_finality: RegenerativeRecoveryGovernanceProposalFinalityV1,
    /// Digest explicitly carried by the proposal action that names this resolution.
    pub proposal_action_resolution_content_digest: String,

    /// Exact threshold-signature subject.
    pub threshold_signature_id: String,
    pub threshold_signature_record_binding: String,
    /// Independent evidence that the signature was cryptographically verified.
    pub threshold_signature_verification_evidence_binding: String,
    /// Digest the threshold signature actually signs.
    pub signed_resolution_content_digest: String,
    pub signature_algorithm: RegenerativeRecoveryThresholdSignatureAlgorithmV1,

    /// Exact committee subject and quorum facts at signing time.
    pub signing_committee_id: String,
    pub signing_committee_record_binding: String,
    pub signing_committee_epoch: u32,
    pub committee_threshold: u32,
    pub signer_count: u32,
    pub committee_scope: RegenerativeRecoveryGovernanceCommitteeScopeV1,
    pub committee_pq_required: bool,
}

impl RegenerativeRecoveryGovernanceAuthorityEvidenceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_RECOVERY_GOVERNANCE_AUTHORITY_SCHEMA_V1 {
            return Err(format!(
                "unsupported recovery governance-authority schema version {}",
                self.schema_version
            ));
        }
        for id in [
            &self.authority_evidence_id,
            &self.threshold_signature_id,
            &self.signing_committee_id,
        ] {
            if !canonical_id(id) {
                return Err("recovery governance-authority identifier is not canonical".into());
            }
        }
        if !self.governance_proposal_id.starts_with("MIP-")
            || !canonical_id(&self.governance_proposal_id)
        {
            return Err("governance proposal ID must be a canonical MIP-* identifier".into());
        }
        for binding in [
            &self.governance_authority_binding,
            &self.resolution_evidence_binding,
            &self.governance_proposal_record_binding,
            &self.threshold_signature_record_binding,
            &self.threshold_signature_verification_evidence_binding,
            &self.signing_committee_record_binding,
        ] {
            if !canonical_reference(binding) {
                return Err("recovery governance-authority binding is not canonical".into());
            }
        }
        for digest in [
            &self.resolution_content_digest,
            &self.proposal_action_resolution_content_digest,
            &self.signed_resolution_content_digest,
        ] {
            if !lower_hex_64(digest) {
                return Err("recovery governance-authority digest must be lowercase 64-hex".into());
            }
        }
        if self.signing_committee_epoch == 0 {
            return Err("signing committee epoch must be positive".into());
        }
        if self.committee_threshold == 0 {
            return Err("signing committee threshold must be positive".into());
        }
        if self.signer_count < self.committee_threshold {
            return Err("threshold signature signer count is below committee threshold".into());
        }
        if self.committee_pq_required
            && self.signature_algorithm
                == RegenerativeRecoveryThresholdSignatureAlgorithmV1::Ecdsa
        {
            return Err("PQ-required committee cannot authorize with ECDSA-only signature".into());
        }
        if !scope_allows_recovery_resolution(&self.committee_scope)? {
            return Err("signing committee scope does not authorize recovery fork resolution".into());
        }
        Ok(())
    }

    pub fn to_payload_json(&self) -> Result<String, String> {
        self.validate()?;
        serde_json::to_string(self)
            .map_err(|error| format!("failed to serialize recovery governance authority: {error}"))
    }

    pub fn content_digest(&self) -> Result<String, String> {
        let payload = self.to_payload_json()?;
        let mut hasher = blake3::Hasher::new();
        hasher.update(b"mycelix-regenerative-recovery-governance-authority-v1\0");
        hasher.update(&(payload.len() as u64).to_le_bytes());
        hasher.update(payload.as_bytes());
        Ok(hasher.finalize().to_hex().to_string())
    }

    pub fn to_maritime_envelope(
        &self,
        platform_id: impl Into<String>,
        generation: u64,
        sequence: u64,
        observed_at_us: u64,
        event_evidence_binding: impl Into<String>,
    ) -> Result<MaritimeEvidenceEnvelope, String> {
        let envelope = MaritimeEvidenceEnvelope::new(
            platform_id,
            generation,
            sequence,
            observed_at_us,
            MaritimeEvidenceKind::AuthorityTransition,
            self.to_payload_json()?,
            event_evidence_binding,
        );
        envelope.validate()?;
        Ok(envelope)
    }
}

pub fn verify_regenerative_recovery_governance_authority(
    resolution: &RegenerativeRecoveryForkResolutionEvidenceV1,
    authority: &RegenerativeRecoveryGovernanceAuthorityEvidenceV1,
) -> Result<(), String> {
    resolution.validate()?;
    authority.validate()?;

    let resolution_digest = resolution.content_digest()?;
    if authority.governance_authority_binding != resolution.governance_authority_binding {
        return Err("governance authority binding does not match fork resolution".into());
    }
    if authority.resolution_evidence_binding != resolution.resolution_evidence_binding {
        return Err("governance authority references a different resolution evidence subject".into());
    }
    if authority.resolution_content_digest != resolution_digest
        || authority.proposal_action_resolution_content_digest != resolution_digest
        || authority.signed_resolution_content_digest != resolution_digest
    {
        return Err("proposal action and threshold signature must bind the exact resolution digest".into());
    }
    Ok(())
}

fn scope_allows_recovery_resolution(
    scope: &RegenerativeRecoveryGovernanceCommitteeScopeV1,
) -> Result<bool, String> {
    match scope {
        RegenerativeRecoveryGovernanceCommitteeScopeV1::All => Ok(true),
        RegenerativeRecoveryGovernanceCommitteeScopeV1::Custom(scope_ids) => {
            if scope_ids.is_empty() {
                return Err("custom signing committee scope cannot be empty".into());
            }
            let mut prior: Option<&str> = None;
            let mut authorized = false;
            for scope_id in scope_ids {
                if !canonical_id(scope_id) {
                    return Err("custom signing committee scope ID is not canonical".into());
                }
                if prior.is_some_and(|value| value >= scope_id.as_str()) {
                    return Err("custom signing committee scope IDs must be sorted and unique".into());
                }
                authorized |= scope_id == RECOVERY_FORK_RESOLUTION_SCOPE_ID_V1;
                prior = Some(scope_id);
            }
            Ok(authorized)
        }
        RegenerativeRecoveryGovernanceCommitteeScopeV1::Constitutional
        | RegenerativeRecoveryGovernanceCommitteeScopeV1::Treasury
        | RegenerativeRecoveryGovernanceCommitteeScopeV1::Protocol => Ok(false),
    }
}

fn canonical_id(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_ID_BYTES
        && value.trim() == value
        && !value.chars().any(char::is_whitespace)
        && !value.chars().any(char::is_control)
}

fn canonical_reference(value: &str) -> bool {
    !value.is_empty()
        && value.len() <= MAX_BINDING_BYTES
        && value.trim() == value
        && value.contains(':')
        && !value.chars().any(char::is_whitespace)
        && !value.chars().any(char::is_control)
}

fn lower_hex_64(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::{
        RegenerativeRecoveryForkResolutionOutcomeV1,
        REGENERATIVE_RECOVERY_FORK_RESOLUTION_SCHEMA_V1,
    };

    fn resolution() -> RegenerativeRecoveryForkResolutionEvidenceV1 {
        RegenerativeRecoveryForkResolutionEvidenceV1 {
            schema_version: REGENERATIVE_RECOVERY_FORK_RESOLUTION_SCHEMA_V1,
            resolution_id: "resolution-1".into(),
            external_recovery_reserve_id: "reserve-test".into(),
            external_recovery_reserve_binding: "reserve:test:evidence".into(),
            viability_evidence_content_digest: "11".repeat(32),
            support_closure_continuity_content_digest: "22".repeat(32),
            fork_spend_sequence: 2,
            fork_predecessor_recovery_evidence_content_digest: Some("33".repeat(32)),
            conflicting_recovery_evidence_content_digests: vec![
                "44".repeat(32),
                "55".repeat(32),
            ],
            outcome: RegenerativeRecoveryForkResolutionOutcomeV1::RetainSelectedBranch,
            selected_recovery_evidence_content_digest: Some("44".repeat(32)),
            fork_observation_evidence_binding: "fork-observation:reserve-test:2".into(),
            governance_authority_binding: "governance-authority:MIP-9001:resolution-1".into(),
            resolution_evidence_binding: "resolution-evidence:reserve-test:2:v1".into(),
        }
    }

    fn authority(
        resolution: &RegenerativeRecoveryForkResolutionEvidenceV1,
    ) -> RegenerativeRecoveryGovernanceAuthorityEvidenceV1 {
        let digest = resolution.content_digest().unwrap();
        RegenerativeRecoveryGovernanceAuthorityEvidenceV1 {
            schema_version: REGENERATIVE_RECOVERY_GOVERNANCE_AUTHORITY_SCHEMA_V1,
            authority_evidence_id: "authority-1".into(),
            governance_authority_binding: resolution.governance_authority_binding.clone(),
            resolution_evidence_binding: resolution.resolution_evidence_binding.clone(),
            resolution_content_digest: digest.clone(),
            governance_proposal_id: "MIP-9001".into(),
            governance_proposal_record_binding: "governance-proposal:MIP-9001:actionhash-1".into(),
            governance_proposal_finality: RegenerativeRecoveryGovernanceProposalFinalityV1::Signed,
            proposal_action_resolution_content_digest: digest.clone(),
            threshold_signature_id: "signature-1".into(),
            threshold_signature_record_binding: "threshold-signature:signature-1:actionhash-2".into(),
            threshold_signature_verification_evidence_binding:
                "threshold-signature-verification:signature-1:v1".into(),
            signed_resolution_content_digest: digest,
            signature_algorithm:
                RegenerativeRecoveryThresholdSignatureAlgorithmV1::HybridEcdsaMlDsa65,
            signing_committee_id: "committee-1".into(),
            signing_committee_record_binding: "signing-committee:committee-1:epoch-7".into(),
            signing_committee_epoch: 7,
            committee_threshold: 3,
            signer_count: 4,
            committee_scope: RegenerativeRecoveryGovernanceCommitteeScopeV1::Custom(vec![
                RECOVERY_FORK_RESOLUTION_SCOPE_ID_V1.into(),
            ]),
            committee_pq_required: true,
        }
    }

    #[test]
    fn signed_governance_authority_must_bind_exact_resolution_digest() {
        let resolution = resolution();
        let authority = authority(&resolution);
        assert_eq!(
            verify_regenerative_recovery_governance_authority(&resolution, &authority),
            Ok(())
        );

        let mut wrong = authority.clone();
        wrong.signed_resolution_content_digest = "aa".repeat(32);
        assert!(verify_regenerative_recovery_governance_authority(&resolution, &wrong).is_err());
    }

    #[test]
    fn signer_count_below_threshold_fails_closed() {
        let resolution = resolution();
        let mut authority = authority(&resolution);
        authority.signer_count = 2;
        assert!(authority.validate().is_err());
    }

    #[test]
    fn pq_required_committee_rejects_ecdsa_only_authority() {
        let resolution = resolution();
        let mut authority = authority(&resolution);
        authority.signature_algorithm = RegenerativeRecoveryThresholdSignatureAlgorithmV1::Ecdsa;
        assert!(authority.validate().is_err());
    }

    #[test]
    fn unrelated_committee_scope_cannot_authorize_recovery_resolution() {
        let resolution = resolution();
        let mut authority = authority(&resolution);
        authority.committee_scope = RegenerativeRecoveryGovernanceCommitteeScopeV1::Protocol;
        assert!(authority.validate().is_err());
    }

    #[test]
    fn custom_scope_must_be_canonical_and_explicitly_include_recovery_resolution() {
        let resolution = resolution();
        let mut authority = authority(&resolution);
        authority.committee_scope = RegenerativeRecoveryGovernanceCommitteeScopeV1::Custom(vec![
            RECOVERY_FORK_RESOLUTION_SCOPE_ID_V1.into(),
            "other-scope".into(),
        ]);
        assert!(authority.validate().is_err());

        authority.committee_scope = RegenerativeRecoveryGovernanceCommitteeScopeV1::Custom(vec![
            "other-scope".into(),
        ]);
        assert!(authority.validate().is_err());
    }

    #[test]
    fn cryptographic_verification_requires_an_external_evidence_binding() {
        let resolution = resolution();
        let mut authority = authority(&resolution);
        authority.threshold_signature_verification_evidence_binding.clear();
        assert!(authority.validate().is_err());
    }
}

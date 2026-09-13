// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Portable composition between one exact recovery-fork resolution and Mycelix's
//! existing threshold-authority theorem.
//!
//! This module deliberately does **not** define another signing committee,
//! threshold, epoch, key, PQ policy, signer set, or cryptographic-verification
//! model. Those semantics already belong to the governance authority stack:
//! `QualifiedThresholdAuthorization` (#69), wrapped as
//! `VerifiedThresholdAuthorization` (#71), with the stable semantic identity
//! registered by #82.
//!
//! Commons proves only the translation/join:
//!
//! exact recovery-fork resolution
//!   -> deterministic exact governance action JSON
//!   -> registered execution-action digest/profile
//!   == exact actions digest/profile carried by the qualified threshold authority
//!   -> exact stable threshold-authorization identity/profile.
//!
//! The opaque evidence bindings in this portable record must be resolved by the
//! governance authority provider. Their mere presence is not cryptographic proof.

use crate::{
    MaritimeEvidenceEnvelope, MaritimeEvidenceKind,
    RegenerativeRecoveryForkResolutionEvidenceV1,
};
use serde::{Deserialize, Serialize};

pub const REGENERATIVE_RECOVERY_GOVERNANCE_AUTHORITY_SCHEMA_V1: u8 = 1;
pub const RECOVERY_GOVERNANCE_ACTION_PROTOCOL_V1: &str =
    "mycelix-regenerative-recovery-governance-action-v1";
pub const GOVERNANCE_ACTIONS_DIGEST_PROFILE_V1: &str =
    "mycelix-governance-execution-authority-v1-blake3-exact-json";
pub const THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE_V1: &str =
    "mycelix-governance-threshold-authorization-v1-blake3-framed-semantic";

const EXECUTION_AUTHORITY_DOMAIN: &[u8] = b"mycelix-governance-execution-authority-v1\0";
const MAX_ID_BYTES: usize = 256;
const MAX_BINDING_BYTES: usize = 2048;
const MAX_ACTION_BYTES: usize = 4096;
const MAX_PROPOSAL_ID_BYTES: usize = 512;

/// Locally derived, exact-byte action subject. This is translation evidence, not
/// governance authority by itself.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct RegenerativeRecoveryGovernanceActionV1 {
    proposal_id: String,
    exact_action_json: String,
    actions_digest: String,
}

impl RegenerativeRecoveryGovernanceActionV1 {
    pub fn proposal_id(&self) -> &str {
        &self.proposal_id
    }

    pub fn exact_action_json(&self) -> &str {
        &self.exact_action_json
    }

    pub fn actions_digest(&self) -> &str {
        &self.actions_digest
    }

    pub const fn actions_digest_profile(&self) -> &'static str {
        GOVERNANCE_ACTIONS_DIGEST_PROFILE_V1
    }

    pub const fn exact_resolution_bound_here(&self) -> bool {
        true
    }

    pub const fn governance_authority_verified_here(&self) -> bool {
        false
    }
}

#[derive(Serialize)]
struct RecoveryResolutionActionWire<'a> {
    protocol_version: &'static str,
    action_type: &'static str,
    resolution_content_digest: &'a str,
    resolution_evidence_binding: &'a str,
}

/// Portable reference to an already-qualified threshold authorization and its #82
/// stable semantic identity. All committee/policy/crypto semantics stay upstream.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct RegenerativeRecoveryGovernanceAuthorityEvidenceV1 {
    pub schema_version: u8,
    pub authority_evidence_id: String,

    /// Exact governance proposal whose action bytes authorize this resolution.
    pub governance_proposal_id: String,

    /// Exact portable bindings to the #69/#71 qualification and #82 stable identity
    /// evidence subjects. Commons preserves these references but does not invent
    /// their authority.
    pub threshold_qualification_evidence_binding: String,
    pub threshold_identity_evidence_binding: String,

    /// Immutable threshold-authorization record/reference used by #71/#82.
    pub threshold_authorization_ref: String,
    pub threshold_authorization_identity_digest: String,
    pub threshold_authorization_identity_profile: String,

    /// Exact action identity echoed by the qualified threshold authorization.
    pub qualified_actions_digest: String,
    pub qualified_actions_digest_profile: String,
}

impl RegenerativeRecoveryGovernanceAuthorityEvidenceV1 {
    pub fn validate(&self) -> Result<(), String> {
        if self.schema_version != REGENERATIVE_RECOVERY_GOVERNANCE_AUTHORITY_SCHEMA_V1 {
            return Err(format!(
                "unsupported recovery governance-authority schema version {}",
                self.schema_version
            ));
        }
        if !canonical_id(&self.authority_evidence_id) {
            return Err("recovery governance-authority evidence ID is not canonical".into());
        }
        if !canonical_proposal_id(&self.governance_proposal_id) {
            return Err("governance proposal ID must be a canonical MIP-* identifier".into());
        }
        for binding in [
            &self.threshold_qualification_evidence_binding,
            &self.threshold_identity_evidence_binding,
            &self.threshold_authorization_ref,
        ] {
            if !canonical_reference(binding) {
                return Err("recovery governance-authority binding is not canonical".into());
            }
        }
        if !lower_hex_64(&self.threshold_authorization_identity_digest)
            || !lower_hex_64(&self.qualified_actions_digest)
        {
            return Err("recovery governance-authority digest is not lowercase 64-hex".into());
        }
        if self.threshold_authorization_identity_profile
            != THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE_V1
        {
            return Err("unexpected threshold-authorization identity profile".into());
        }
        if self.qualified_actions_digest_profile != GOVERNANCE_ACTIONS_DIGEST_PROFILE_V1 {
            return Err("unexpected governance actions-digest profile".into());
        }
        Ok(())
    }

    /// Deterministic authority binding that the fork-resolution record must name.
    /// A bare caller-chosen alias cannot substitute for the stable threshold identity.
    pub fn canonical_authority_binding(&self) -> Result<String, String> {
        self.validate()?;
        Ok(format!(
            "threshold-authorization:{}:{}",
            self.threshold_authorization_identity_profile,
            self.threshold_authorization_identity_digest
        ))
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

/// Translate the exact recovery resolution into the exact-byte governance action
/// domain used by Mycelix execution authority. This mirrors the registered
/// `mycelix-execution-action-digest` rule so whitespace/key-order changes remain
/// authority-significant until these lineages converge on the shared crate.
pub fn qualify_regenerative_recovery_governance_action(
    proposal_id: &str,
    resolution: &RegenerativeRecoveryForkResolutionEvidenceV1,
) -> Result<RegenerativeRecoveryGovernanceActionV1, String> {
    if !canonical_proposal_id(proposal_id) || proposal_id.len() > MAX_PROPOSAL_ID_BYTES {
        return Err("invalid governance proposal ID for recovery action".into());
    }
    resolution.validate()?;
    let resolution_content_digest = resolution.content_digest()?;
    let wire = RecoveryResolutionActionWire {
        protocol_version: RECOVERY_GOVERNANCE_ACTION_PROTOCOL_V1,
        action_type: "resolve_regenerative_recovery_fork",
        resolution_content_digest: &resolution_content_digest,
        resolution_evidence_binding: &resolution.resolution_evidence_binding,
    };
    let exact_action_json = serde_json::to_string(&wire)
        .map_err(|error| format!("failed to serialize recovery governance action: {error}"))?;
    if exact_action_json.len() > MAX_ACTION_BYTES {
        return Err("recovery governance action exceeds registered action-byte limit".into());
    }
    let actions_digest = execution_authority_digest(proposal_id, &exact_action_json)?;
    Ok(RegenerativeRecoveryGovernanceActionV1 {
        proposal_id: proposal_id.to_string(),
        exact_action_json,
        actions_digest,
    })
}

/// Compose one exact fork resolution with an already-qualified threshold authority.
/// This verifies the semantic join; it does not re-run #69/#71/#82 provider logic.
pub fn verify_regenerative_recovery_governance_authority(
    resolution: &RegenerativeRecoveryForkResolutionEvidenceV1,
    authority: &RegenerativeRecoveryGovernanceAuthorityEvidenceV1,
) -> Result<RegenerativeRecoveryGovernanceActionV1, String> {
    resolution.validate()?;
    authority.validate()?;

    if resolution.governance_authority_binding != authority.canonical_authority_binding()? {
        return Err("fork resolution does not name the exact stable threshold authorization".into());
    }

    let action = qualify_regenerative_recovery_governance_action(
        &authority.governance_proposal_id,
        resolution,
    )?;
    if action.actions_digest != authority.qualified_actions_digest
        || action.actions_digest_profile() != authority.qualified_actions_digest_profile
    {
        return Err("qualified threshold authority does not bind the exact recovery action".into());
    }
    Ok(action)
}

fn execution_authority_digest(proposal_id: &str, actions: &str) -> Result<String, String> {
    if proposal_id.trim().is_empty() || proposal_id.len() > MAX_PROPOSAL_ID_BYTES {
        return Err("invalid proposal ID for governance action digest".into());
    }
    if actions.is_empty() || actions.len() > MAX_ACTION_BYTES {
        return Err("invalid governance action byte length".into());
    }
    let mut hasher = blake3::Hasher::new();
    hasher.update(EXECUTION_AUTHORITY_DOMAIN);
    hasher.update(&(proposal_id.len() as u64).to_le_bytes());
    hasher.update(proposal_id.as_bytes());
    hasher.update(&(actions.len() as u64).to_le_bytes());
    hasher.update(actions.as_bytes());
    Ok(hasher.finalize().to_hex().to_string())
}

fn canonical_proposal_id(value: &str) -> bool {
    value.starts_with("MIP-") && canonical_id(value)
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

    fn authority_binding(identity_digest: &str) -> String {
        format!(
            "threshold-authorization:{}:{}",
            THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE_V1,
            identity_digest
        )
    }

    fn resolution(identity_digest: &str) -> RegenerativeRecoveryForkResolutionEvidenceV1 {
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
            governance_authority_binding: authority_binding(identity_digest),
            resolution_evidence_binding: "resolution-evidence:reserve-test:2:v1".into(),
        }
    }

    fn evidence(
        resolution: &RegenerativeRecoveryForkResolutionEvidenceV1,
        identity_digest: &str,
    ) -> RegenerativeRecoveryGovernanceAuthorityEvidenceV1 {
        let action = qualify_regenerative_recovery_governance_action("MIP-9001", resolution)
            .unwrap();
        RegenerativeRecoveryGovernanceAuthorityEvidenceV1 {
            schema_version: REGENERATIVE_RECOVERY_GOVERNANCE_AUTHORITY_SCHEMA_V1,
            authority_evidence_id: "authority-1".into(),
            governance_proposal_id: "MIP-9001".into(),
            threshold_qualification_evidence_binding:
                "mycelix-governance-threshold-qualification:receipt-1".into(),
            threshold_identity_evidence_binding:
                "mycelix-governance-threshold-identity:receipt-1".into(),
            threshold_authorization_ref: "threshold-authorization:record-1".into(),
            threshold_authorization_identity_digest: identity_digest.into(),
            threshold_authorization_identity_profile:
                THRESHOLD_AUTHORIZATION_IDENTITY_PROFILE_V1.into(),
            qualified_actions_digest: action.actions_digest().into(),
            qualified_actions_digest_profile: GOVERNANCE_ACTIONS_DIGEST_PROFILE_V1.into(),
        }
    }

    #[test]
    fn exact_resolution_translates_into_exact_qualified_action_domain() {
        let identity = "66".repeat(32);
        let resolution = resolution(&identity);
        let authority = evidence(&resolution, &identity);
        let action = verify_regenerative_recovery_governance_authority(
            &resolution,
            &authority,
        )
        .unwrap();
        assert!(action.exact_resolution_bound_here());
        assert!(!action.governance_authority_verified_here());
        assert_eq!(action.actions_digest(), authority.qualified_actions_digest);
        assert_eq!(action.actions_digest_profile(), GOVERNANCE_ACTIONS_DIGEST_PROFILE_V1);
    }

    #[test]
    fn changed_resolution_changes_governance_action_and_fails_old_authority() {
        let identity = "66".repeat(32);
        let original = resolution(&identity);
        let authority = evidence(&original, &identity);
        let mut changed = original.clone();
        changed.selected_recovery_evidence_content_digest = Some("55".repeat(32));
        assert!(verify_regenerative_recovery_governance_authority(
            &changed,
            &authority,
        )
        .is_err());
    }

    #[test]
    fn caller_alias_cannot_substitute_for_stable_threshold_identity() {
        let identity = "66".repeat(32);
        let mut resolution = resolution(&identity);
        let authority = evidence(&resolution, &identity);
        resolution.governance_authority_binding = "governance:looks-valid:v1".into();
        assert!(verify_regenerative_recovery_governance_authority(
            &resolution,
            &authority,
        )
        .is_err());
    }

    #[test]
    fn action_digest_or_profile_substitution_fails_closed() {
        let identity = "66".repeat(32);
        let resolution = resolution(&identity);
        let mut authority = evidence(&resolution, &identity);
        authority.qualified_actions_digest = "aa".repeat(32);
        assert!(verify_regenerative_recovery_governance_authority(
            &resolution,
            &authority,
        )
        .is_err());

        let mut wrong_profile = evidence(&resolution, &identity);
        wrong_profile.qualified_actions_digest_profile = "profile:other".into();
        assert!(wrong_profile.validate().is_err());
    }

    #[test]
    fn threshold_identity_profile_is_exact_not_caller_selected() {
        let identity = "66".repeat(32);
        let resolution = resolution(&identity);
        let mut authority = evidence(&resolution, &identity);
        authority.threshold_authorization_identity_profile = "profile:other".into();
        assert!(authority.validate().is_err());
    }

    #[test]
    fn governance_action_serialization_is_deterministic() {
        let identity = "66".repeat(32);
        let resolution = resolution(&identity);
        let a = qualify_regenerative_recovery_governance_action("MIP-9001", &resolution)
            .unwrap();
        let b = qualify_regenerative_recovery_governance_action("MIP-9001", &resolution)
            .unwrap();
        assert_eq!(a.exact_action_json(), b.exact_action_json());
        assert_eq!(a.actions_digest(), b.actions_digest());
    }
}

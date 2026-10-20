// Copyright (C) 2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Policy-release lifecycle checks performed *after* an authenticated distribution
//! adapter (for example, a TUF client) has verified the metadata chain.
//!
//! This module does not verify signatures, establish trusted time, fetch revocation
//! state, or persist anti-rollback state. Its distribution status and trusted context
//! must be supplied by a separately trusted adapter, never parsed from the candidate
//! policy request. A receipt does not grant admission or action authority.

use super::{sha256, validate_json_against_schema};
use serde::{Deserialize, Serialize};

const RELEASE_SCHEMA: &[u8] =
    include_bytes!("../schemas/claim-policy-release-descriptor-v1.schema.json");

pub const RELEASE_DESCRIPTOR_SCHEMA_VERSION: &str =
    "mycelix.claim-policy-release-descriptor.v1";

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PolicyReleaseDescriptorV1 {
    pub schema_version: String,
    pub policy_id: String,
    pub audience: String,
    pub version: u64,
    pub revocation_epoch: u64,
    /// Digest recorded in authenticated distribution metadata for the profile target.
    pub target_profile_sha256: String,
    pub policy_profile_schema_sha256: String,
    pub issued_at_unix_seconds: i64,
    pub expires_at_unix_seconds: i64,
}

/// Outcome returned by an independently authenticated distribution verifier.
/// This value must never be accepted from the untrusted candidate request.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum AuthenticatedDistributionStatus {
    VerifiedCurrent,
    InvalidSignature,
    UnknownPublisherKey,
    RevokedPublisherKey,
    MetadataExpired,
    MetadataStale,
    RollbackDetected,
    MixAndMatchDetected,
    SignatureThresholdNotMet,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct InstalledPolicyHeadV1 {
    pub version: u64,
    pub target_profile_sha256: String,
    pub revocation_epoch: u64,
}

/// Trusted context supplied by the integration boundary, not by the policy candidate.
/// The integration is responsible for authenticating its policy/audience configuration,
/// trusted-time source, revocation epoch, distribution status, and persisted head state.
#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PolicyLifecycleTrustedContextV1 {
    pub expected_policy_id: String,
    pub expected_audience: String,
    pub expected_policy_profile_schema_sha256: String,
    pub minimum_revocation_epoch: u64,
    pub trusted_time_unix_seconds: Option<i64>,
    pub distribution_status: AuthenticatedDistributionStatus,
    pub installed_head: Option<InstalledPolicyHeadV1>,
}

#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PolicyLifecycleViolationV1 {
    pub code: String,
    pub value: String,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum PolicyLifecycleDispositionV1 {
    AcceptedNewRelease,
    AlreadyCurrent,
    Rejected,
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct PolicyLifecycleReceiptV1 {
    pub schema_version: String,
    pub policy_id: String,
    pub audience: String,
    pub release_descriptor_sha256: String,
    pub actual_profile_sha256: String,
    pub installed_head_before: Option<InstalledPolicyHeadV1>,
    /// Proposed next head only. The caller must persist it atomically after all
    /// required checks; emitting this field does not mutate durable anti-rollback state.
    pub proposed_next_head: Option<InstalledPolicyHeadV1>,
    pub distribution_status: AuthenticatedDistributionStatus,
    pub disposition: PolicyLifecycleDispositionV1,
    pub violations: Vec<PolicyLifecycleViolationV1>,
    pub policy_lifecycle_only: bool,
    pub evidence_admission_granted: bool,
    pub action_authority_granted: bool,
    pub nonclaims: Vec<String>,
}

const NONCLAIMS: &[&str] = &[
    "This evaluator assumes an upstream distribution verifier supplied the authenticated metadata status.",
    "This evaluator does not authenticate policy publishers, signatures, trust roots, trusted time, or revocation state.",
    "A caller must persist anti-rollback state atomically in trusted storage; a receipt does not persist state.",
    "Policy lifecycle acceptance is not evidence truth, canonical evidence admission, user consent, or action authority.",
    "The claim-ceiling policy-fit gate remains a separate required check on the profile contents.",
];

fn add_violation(violations: &mut Vec<PolicyLifecycleViolationV1>, code: &str, value: String) {
    violations.push(PolicyLifecycleViolationV1 {
        code: code.to_owned(),
        value,
    });
}

fn status_violation(status: AuthenticatedDistributionStatus) -> Option<(&'static str, &'static str)> {
    match status {
        AuthenticatedDistributionStatus::VerifiedCurrent => None,
        AuthenticatedDistributionStatus::InvalidSignature => {
            Some(("metadata_signature_invalid", "distribution metadata signature is invalid"))
        }
        AuthenticatedDistributionStatus::UnknownPublisherKey => {
            Some(("publisher_key_unknown", "distribution publisher key is not trusted"))
        }
        AuthenticatedDistributionStatus::RevokedPublisherKey => {
            Some(("publisher_key_revoked", "distribution publisher key is revoked"))
        }
        AuthenticatedDistributionStatus::MetadataExpired => {
            Some(("metadata_expired", "authenticated distribution metadata is expired"))
        }
        AuthenticatedDistributionStatus::MetadataStale => {
            Some(("metadata_stale", "distribution metadata is not current"))
        }
        AuthenticatedDistributionStatus::RollbackDetected => {
            Some(("distribution_rollback_detected", "distribution verifier detected rollback"))
        }
        AuthenticatedDistributionStatus::MixAndMatchDetected => {
            Some(("distribution_mix_and_match_detected", "distribution metadata is inconsistent"))
        }
        AuthenticatedDistributionStatus::SignatureThresholdNotMet => {
            Some(("signature_threshold_not_met", "required signature threshold was not met"))
        }
    }
}

fn valid_sha256(value: &str) -> bool {
    value.len() == 64
        && value
            .bytes()
            .all(|byte| byte.is_ascii_digit() || (b'a'..=b'f').contains(&byte))
}

/// Evaluate release identity, target digest, validity, revocation epoch, and monotonic
/// versioning. The profile bytes must be the exact bytes fetched by the trusted adapter.
/// The trusted context must come from a separately authenticated configuration/state path.
pub fn evaluate_policy_release_bytes(
    release_bytes: &[u8],
    profile_bytes: &[u8],
    trusted_context: &PolicyLifecycleTrustedContextV1,
) -> Result<PolicyLifecycleReceiptV1, String> {
    validate_json_against_schema(RELEASE_SCHEMA, release_bytes, "policy release descriptor")?;
    let release: PolicyReleaseDescriptorV1 = serde_json::from_slice(release_bytes)
        .map_err(|error| format!("policy release descriptor deserialization failed: {error}"))?;

    if release.schema_version != RELEASE_DESCRIPTOR_SCHEMA_VERSION {
        return Err(format!(
            "unsupported policy release descriptor schema version: {}",
            release.schema_version
        ));
    }
    if release.policy_id.trim().is_empty() || release.audience.trim().is_empty() {
        return Err("policy_id and audience must not be empty".to_owned());
    }
    if !valid_sha256(&release.target_profile_sha256)
        || !valid_sha256(&release.policy_profile_schema_sha256)
        || !valid_sha256(&trusted_context.expected_policy_profile_schema_sha256)
    {
        return Err("policy/profile schema digests must be 64 lowercase hexadecimal characters".to_owned());
    }
    if trusted_context.expected_policy_id.trim().is_empty()
        || trusted_context.expected_audience.trim().is_empty()
    {
        return Err("trusted policy identity and audience must not be empty".to_owned());
    }

    let actual_profile_sha256 = sha256(profile_bytes);
    let mut violations = Vec::new();

    if let Some((code, message)) = status_violation(trusted_context.distribution_status) {
        add_violation(&mut violations, code, message.to_owned());
    }
    if release.policy_id != trusted_context.expected_policy_id {
        add_violation(
            &mut violations,
            "policy_id_mismatch",
            format!("expected {}, received {}", trusted_context.expected_policy_id, release.policy_id),
        );
    }
    if release.audience != trusted_context.expected_audience {
        add_violation(
            &mut violations,
            "policy_audience_mismatch",
            format!("expected {}, received {}", trusted_context.expected_audience, release.audience),
        );
    }
    if release.policy_profile_schema_sha256
        != trusted_context.expected_policy_profile_schema_sha256
    {
        add_violation(
            &mut violations,
            "policy_schema_digest_mismatch",
            format!(
                "expected {}, received {}",
                trusted_context.expected_policy_profile_schema_sha256,
                release.policy_profile_schema_sha256
            ),
        );
    }
    if actual_profile_sha256 != release.target_profile_sha256 {
        add_violation(
            &mut violations,
            "profile_target_digest_mismatch",
            format!(
                "descriptor expected {}, actual profile bytes hash to {}",
                release.target_profile_sha256, actual_profile_sha256
            ),
        );
    }
    if release.issued_at_unix_seconds >= release.expires_at_unix_seconds {
        add_violation(
            &mut violations,
            "invalid_release_interval",
            format!(
                "issued_at {} must be before expires_at {}",
                release.issued_at_unix_seconds, release.expires_at_unix_seconds
            ),
        );
    }
    if release.revocation_epoch < trusted_context.minimum_revocation_epoch {
        add_violation(
            &mut violations,
            "revocation_epoch_stale",
            format!(
                "required at least {}, received {}",
                trusted_context.minimum_revocation_epoch, release.revocation_epoch
            ),
        );
    }

    match trusted_context.trusted_time_unix_seconds {
        None => add_violation(
            &mut violations,
            "trusted_time_unavailable",
            "no trusted time was supplied by the integration boundary".to_owned(),
        ),
        Some(now) => {
            if now < release.issued_at_unix_seconds {
                add_violation(
                    &mut violations,
                    "release_not_yet_valid",
                    format!("trusted time {now} precedes issue time {}", release.issued_at_unix_seconds),
                );
            }
            if now >= release.expires_at_unix_seconds {
                add_violation(
                    &mut violations,
                    "release_expired",
                    format!("trusted time {now} is at or after expiry {}", release.expires_at_unix_seconds),
                );
            }
        }
    }

    let mut already_current = false;
    if let Some(installed) = &trusted_context.installed_head {
        if release.version < installed.version {
            add_violation(
                &mut violations,
                "policy_version_rollback",
                format!("installed version {}, candidate version {}", installed.version, release.version),
            );
        } else if release.version == installed.version {
            if release.target_profile_sha256 != installed.target_profile_sha256
                || release.revocation_epoch != installed.revocation_epoch
            {
                add_violation(
                    &mut violations,
                    "policy_version_equivocation",
                    format!(
                        "version {} conflicts with the installed digest/revocation epoch",
                        release.version
                    ),
                );
            } else {
                already_current = true;
            }
        }
    }

    violations.sort();
    violations.dedup();
    let disposition = if !violations.is_empty() {
        PolicyLifecycleDispositionV1::Rejected
    } else if already_current {
        PolicyLifecycleDispositionV1::AlreadyCurrent
    } else {
        PolicyLifecycleDispositionV1::AcceptedNewRelease
    };
    let proposed_next_head = match disposition {
        PolicyLifecycleDispositionV1::AcceptedNewRelease => Some(InstalledPolicyHeadV1 {
            version: release.version,
            target_profile_sha256: release.target_profile_sha256.clone(),
            revocation_epoch: release.revocation_epoch,
        }),
        PolicyLifecycleDispositionV1::AlreadyCurrent => trusted_context.installed_head.clone(),
        PolicyLifecycleDispositionV1::Rejected => None,
    };

    Ok(PolicyLifecycleReceiptV1 {
        schema_version: "mycelix.claim-policy-lifecycle.receipt.v1".to_owned(),
        policy_id: release.policy_id,
        audience: release.audience,
        release_descriptor_sha256: sha256(release_bytes),
        actual_profile_sha256,
        installed_head_before: trusted_context.installed_head.clone(),
        proposed_next_head,
        distribution_status: trusted_context.distribution_status,
        disposition,
        violations,
        policy_lifecycle_only: true,
        evidence_admission_granted: false,
        action_authority_granted: false,
        nonclaims: NONCLAIMS.iter().map(|value| (*value).to_owned()).collect(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use serde::Deserialize;
    use serde_json::json;

    const PROFILE_BYTES: &[u8] = b"{\"policy\":\"sample-profile\"}";
    const PROFILE_SCHEMA_SHA256: &str =
        "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa";

    fn release() -> PolicyReleaseDescriptorV1 {
        PolicyReleaseDescriptorV1 {
            schema_version: RELEASE_DESCRIPTOR_SCHEMA_VERSION.to_owned(),
            policy_id: "mycelix.policy.instrument.research".to_owned(),
            audience: "mycelix.claim-ceiling.policy-check".to_owned(),
            version: 2,
            revocation_epoch: 3,
            target_profile_sha256: sha256(PROFILE_BYTES),
            policy_profile_schema_sha256: PROFILE_SCHEMA_SHA256.to_owned(),
            issued_at_unix_seconds: 100,
            expires_at_unix_seconds: 300,
        }
    }

    fn context() -> PolicyLifecycleTrustedContextV1 {
        PolicyLifecycleTrustedContextV1 {
            expected_policy_id: "mycelix.policy.instrument.research".to_owned(),
            expected_audience: "mycelix.claim-ceiling.policy-check".to_owned(),
            expected_policy_profile_schema_sha256: PROFILE_SCHEMA_SHA256.to_owned(),
            minimum_revocation_epoch: 3,
            trusted_time_unix_seconds: Some(200),
            distribution_status: AuthenticatedDistributionStatus::VerifiedCurrent,
            installed_head: Some(InstalledPolicyHeadV1 {
                version: 1,
                target_profile_sha256: "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".to_owned(),
                revocation_epoch: 3,
            }),
        }
    }

    fn evaluate(
        release: PolicyReleaseDescriptorV1,
        context: PolicyLifecycleTrustedContextV1,
        profile_bytes: &[u8],
    ) -> PolicyLifecycleReceiptV1 {
        evaluate_policy_release_bytes(
            &serde_json::to_vec(&release).expect("serialize release"),
            profile_bytes,
            &context,
        )
        .expect("well-formed lifecycle inputs should return a receipt")
    }

    #[test]
    fn accepts_a_new_current_release_and_proposes_monotonic_state() {
        let receipt = evaluate(release(), context(), PROFILE_BYTES);
        assert_eq!(receipt.disposition, PolicyLifecycleDispositionV1::AcceptedNewRelease);
        assert_eq!(receipt.proposed_next_head.as_ref().unwrap().version, 2);
        assert!(!receipt.evidence_admission_granted);
        assert!(!receipt.action_authority_granted);
    }

    #[test]
    fn an_exact_current_release_is_idempotent_without_state_advance() {
        let mut ctx = context();
        let current = release();
        ctx.installed_head = Some(InstalledPolicyHeadV1 {
            version: current.version,
            target_profile_sha256: current.target_profile_sha256.clone(),
            revocation_epoch: current.revocation_epoch,
        });
        let receipt = evaluate(current, ctx.clone(), PROFILE_BYTES);
        assert_eq!(receipt.disposition, PolicyLifecycleDispositionV1::AlreadyCurrent);
        assert_eq!(receipt.proposed_next_head, ctx.installed_head);
    }

    #[test]
    fn rejects_a_lower_release_version_as_rollback() {
        let mut candidate = release();
        candidate.version = 1;
        let receipt = evaluate(candidate, context(), PROFILE_BYTES);
        assert_eq!(receipt.disposition, PolicyLifecycleDispositionV1::Rejected);
        assert!(receipt.violations.iter().any(|v| v.code == "policy_version_rollback"));
        assert!(receipt.proposed_next_head.is_none());
    }

    #[test]
    fn rejects_same_version_with_different_content_or_epoch_as_equivocation() {
        let ctx = context();
        let mut candidate = release();
        let mut conflict_ctx = ctx.clone();
        conflict_ctx.installed_head = Some(InstalledPolicyHeadV1 {
            version: candidate.version,
            target_profile_sha256: "cccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccccc".to_owned(),
            revocation_epoch: candidate.revocation_epoch,
        });
        let receipt = evaluate(candidate.clone(), conflict_ctx, PROFILE_BYTES);
        assert!(receipt.violations.iter().any(|v| v.code == "policy_version_equivocation"));

        let mut epoch_conflict_ctx = ctx;
        epoch_conflict_ctx.installed_head = Some(InstalledPolicyHeadV1 {
            version: candidate.version,
            target_profile_sha256: candidate.target_profile_sha256.clone(),
            revocation_epoch: candidate.revocation_epoch,
        });
        candidate.revocation_epoch += 1;
        let receipt = evaluate(candidate, epoch_conflict_ctx, PROFILE_BYTES);
        assert!(receipt.violations.iter().any(|v| v.code == "policy_version_equivocation"));
    }

    #[test]
    fn rejects_profile_byte_tampering_and_wrong_audience() {
        let mut candidate = release();
        candidate.audience = "other.consumer".to_owned();
        let receipt = evaluate(candidate, context(), b"tampered bytes");
        let codes: Vec<_> = receipt.violations.iter().map(|v| v.code.as_str()).collect();
        assert!(codes.contains(&"policy_audience_mismatch"));
        assert!(codes.contains(&"profile_target_digest_mismatch"));
    }

    #[test]
    fn rejects_unknown_or_revoked_publisher_and_expired_distribution_metadata() {
        let statuses = [
            (AuthenticatedDistributionStatus::UnknownPublisherKey, "publisher_key_unknown"),
            (AuthenticatedDistributionStatus::RevokedPublisherKey, "publisher_key_revoked"),
            (AuthenticatedDistributionStatus::MetadataExpired, "metadata_expired"),
            (AuthenticatedDistributionStatus::RollbackDetected, "distribution_rollback_detected"),
            (AuthenticatedDistributionStatus::MixAndMatchDetected, "distribution_mix_and_match_detected"),
        ];
        for (status, code) in statuses {
            let mut ctx = context();
            ctx.distribution_status = status;
            let receipt = evaluate(release(), ctx, PROFILE_BYTES);
            assert_eq!(receipt.disposition, PolicyLifecycleDispositionV1::Rejected);
            assert!(receipt.violations.iter().any(|v| v.code == code), "{code}");
        }
    }

    #[test]
    fn rejects_stale_revocation_epoch_and_expired_release() {
        let mut ctx = context();
        ctx.minimum_revocation_epoch = 4;
        ctx.trusted_time_unix_seconds = Some(300);
        let receipt = evaluate(release(), ctx, PROFILE_BYTES);
        let codes: Vec<_> = receipt.violations.iter().map(|v| v.code.as_str()).collect();
        assert!(codes.contains(&"revocation_epoch_stale"));
        assert!(codes.contains(&"release_expired"));
    }

    #[test]
    fn missing_trusted_time_fails_closed() {
        let mut ctx = context();
        ctx.trusted_time_unix_seconds = None;
        let receipt = evaluate(release(), ctx, PROFILE_BYTES);
        assert_eq!(receipt.disposition, PolicyLifecycleDispositionV1::Rejected);
        assert!(receipt.violations.iter().any(|v| v.code == "trusted_time_unavailable"));
    }

    #[test]
    fn rejects_release_issued_in_the_future_and_invalid_intervals() {
        let mut candidate = release();
        candidate.issued_at_unix_seconds = 250;
        candidate.expires_at_unix_seconds = 250;
        let receipt = evaluate(candidate, context(), PROFILE_BYTES);
        let codes: Vec<_> = receipt.violations.iter().map(|v| v.code.as_str()).collect();
        assert!(codes.contains(&"invalid_release_interval"));
        assert!(codes.contains(&"release_not_yet_valid"));
    }

    #[test]
    fn all_distribution_failures_are_rejections() {
        for status in [
            AuthenticatedDistributionStatus::InvalidSignature,
            AuthenticatedDistributionStatus::UnknownPublisherKey,
            AuthenticatedDistributionStatus::RevokedPublisherKey,
            AuthenticatedDistributionStatus::MetadataExpired,
            AuthenticatedDistributionStatus::MetadataStale,
            AuthenticatedDistributionStatus::RollbackDetected,
            AuthenticatedDistributionStatus::MixAndMatchDetected,
            AuthenticatedDistributionStatus::SignatureThresholdNotMet,
        ] {
            let mut ctx = context();
            ctx.distribution_status = status;
            assert_eq!(
                evaluate(release(), ctx, PROFILE_BYTES).disposition,
                PolicyLifecycleDispositionV1::Rejected,
                "distribution status {status:?} must not admit a release"
            );
        }
    }

    #[test]
    fn release_descriptor_rejects_unknown_fields_and_invalid_schema() {
        let mut value = serde_json::to_value(release()).unwrap();
        value["action_authority_granted"] = json!(true);
        let bytes = serde_json::to_vec(&value).unwrap();
        assert!(evaluate_policy_release_bytes(&bytes, PROFILE_BYTES, &context()).is_err());

        let schema: serde_json::Value = serde_json::from_slice(RELEASE_SCHEMA).unwrap();
        jsonschema::JSONSchema::options()
            .with_draft(jsonschema::Draft::Draft202012)
            .compile(&schema)
            .expect("release schema must be valid Draft 2020-12");
    }
    #[derive(Debug, Deserialize)]
    struct LifecycleVectorCorpus {
        schema_version: String,
        cases: Vec<LifecycleVectorCase>,
    }

    #[derive(Debug, Deserialize)]
    struct LifecycleVectorCase {
        id: String,
        release: PolicyReleaseDescriptorV1,
        profile_bytes: String,
        trusted_context: PolicyLifecycleTrustedContextV1,
        expected_disposition: PolicyLifecycleDispositionV1,
        expected_violation_codes: Vec<String>,
    }

    #[test]
    fn frozen_lifecycle_attack_vectors_match_reference_state_machine() {
        let bytes: &[u8] =
            include_bytes!("../fixtures/claim-policy-lifecycle-golden-v1.json");
        let corpus: LifecycleVectorCorpus =
            serde_json::from_slice(bytes).expect("lifecycle vector corpus must parse");
        assert_eq!(
            corpus.schema_version,
            "mycelix.claim-policy-lifecycle.golden.v1"
        );
        assert_eq!(corpus.cases.len(), 19);

        for vector in corpus.cases {
            let receipt = evaluate_policy_release_bytes(
                &serde_json::to_vec(&vector.release).expect("serialize descriptor"),
                vector.profile_bytes.as_bytes(),
                &vector.trusted_context,
            )
            .unwrap_or_else(|error| panic!("vector {} failed to evaluate: {error}", vector.id));

            assert_eq!(
                receipt.disposition, vector.expected_disposition,
                "wrong disposition for vector {}",
                vector.id
            );
            let mut actual: Vec<_> = receipt
                .violations
                .iter()
                .map(|item| item.code.clone())
                .collect();
            let mut expected = vector.expected_violation_codes;
            actual.sort();
            expected.sort();
            assert_eq!(actual, expected, "wrong violation codes for vector {}", vector.id);
            if receipt.disposition == PolicyLifecycleDispositionV1::Rejected {
                assert!(receipt.proposed_next_head.is_none(), "rejected vector {} proposed state", vector.id);
            }
            assert!(!receipt.evidence_admission_granted);
            assert!(!receipt.action_authority_granted);
        }
    }

}

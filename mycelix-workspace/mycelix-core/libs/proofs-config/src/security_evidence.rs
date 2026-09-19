// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//
//! Typed proof-security targets and measured evidence.
//!
//! This module deliberately does not decide whether a proof theorem is sound,
//! whether a witness is private, or whether an application should authorize an
//! action. It compares cryptographic security measurements against an explicit
//! target while preserving their provenance.

use serde::{Deserialize, Serialize};

use crate::SecurityLevel;

/// Exact cryptographic profile identity for one measured proof lineage.
///
/// These fields are evaluated as one tuple. Independent allowlists would permit
/// a "Frankenstein" combination of individually allowed backend/hash/field/options
/// values that had never actually been measured together.
#[derive(Clone, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub struct ProofSecurityProfileIdentityV1 {
    pub statement_profile: String,
    pub backend_profile: String,
    pub backend_version: String,
    pub proof_options_profile: String,
    pub hash_profile: String,
    pub field_profile: String,
}

/// Exact policy target for measured proof-security evidence.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct ProofSecurityTargetV1 {
    /// Stable target identifier chosen by the deployment/policy layer.
    pub profile_id: String,
    /// Existing Mycelix policy tier. This is intent, not measured security.
    pub policy_tier: SecurityLevel,
    /// Exact evidence schema version when the target requires one.
    pub required_evidence_version: Option<u32>,
    /// Minimum conjectured security when required.
    pub min_conjectured_bits: Option<u32>,
    /// Minimum proven unique-decoding security when required.
    pub min_proven_udr_bits: Option<u32>,
    /// Minimum proven list-decoding security when required.
    pub min_proven_ldr_bits: Option<u32>,
    /// Minimum collision-security ceiling required of the selected hash profile.
    pub min_hash_collision_bits: u32,
    /// Exact allowed cryptographic profile tuples.
    ///
    /// An empty list fails closed. Security-bearing profile identity must be an
    /// explicit policy decision rather than an implicit wildcard.
    pub allowed_profiles: Vec<ProofSecurityProfileIdentityV1>,
}

/// Measured security evidence for one exact proof/backend/profile lineage.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct MeasuredProofSecurityV1 {
    pub evidence_version: u32,
    pub statement_profile: String,
    pub backend_profile: String,
    pub backend_version: String,
    pub proof_options_profile: String,
    pub hash_profile: String,
    pub field_profile: String,

    /// Estimator values are optional because absence must remain distinguishable
    /// from a measured value of zero.
    pub conjectured_bits: Option<u32>,
    pub proven_udr_bits: Option<u32>,
    pub proven_ldr_bits: Option<u32>,
    pub hash_collision_bits: Option<u32>,

    pub subject_sha: String,
    pub dependency_graph_digest: String,
    pub measurement_receipt_digest: String,
}

impl MeasuredProofSecurityV1 {
    /// Return the exact security-bearing profile tuple represented by this
    /// measurement. Lineage digests and subject identity are intentionally kept
    /// separate for later theorem/receipt admission.
    pub fn profile_identity(&self) -> ProofSecurityProfileIdentityV1 {
        ProofSecurityProfileIdentityV1 {
            statement_profile: self.statement_profile.clone(),
            backend_profile: self.backend_profile.clone(),
            backend_version: self.backend_version.clone(),
            proof_options_profile: self.proof_options_profile.clone(),
            hash_profile: self.hash_profile.clone(),
            field_profile: self.field_profile.clone(),
        }
    }
}

/// Why measured evidence did not satisfy a security target.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum SecurityTargetFailureV1 {
    NoAllowedSecurityProfiles,
    EvidenceVersionMismatch { required: u32, actual: u32 },
    SecurityProfileNotAllowed {
        actual: ProofSecurityProfileIdentityV1,
    },
    MissingConjecturedMeasurement,
    MissingProvenUdrMeasurement,
    MissingProvenLdrMeasurement,
    MissingHashCollisionMeasurement,
    ConjecturedBelowTarget { required: u32, effective: u32 },
    ProvenUdrBelowTarget { required: u32, effective: u32 },
    ProvenLdrBelowTarget { required: u32, effective: u32 },
    HashCollisionBelowTarget { required: u32, measured: u32 },
}

/// Result of comparing measured cryptographic evidence with a policy target.
///
/// This is not a production-authorization receipt. AIR/theorem qualification,
/// evidence-lineage authentication, revocation and application policy remain
/// separate inputs to a later admission layer.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct SecurityTargetEvaluationV1 {
    pub target_profile_id: String,
    pub evidence_subject_sha: String,
    pub evaluated_profile: ProofSecurityProfileIdentityV1,
    pub effective_conjectured_bits: Option<u32>,
    pub effective_proven_udr_bits: Option<u32>,
    pub effective_proven_ldr_bits: Option<u32>,
    pub failures: Vec<SecurityTargetFailureV1>,
}

impl SecurityTargetEvaluationV1 {
    /// Whether the measured cryptographic evidence satisfies this target.
    ///
    /// This says nothing about theorem soundness, witness privacy, receipt
    /// authenticity, revocation status, or application authority.
    pub fn meets_target(&self) -> bool {
        self.failures.is_empty()
    }
}

/// Compare exact measured security evidence against an explicit policy target.
///
/// Every estimator is capped by the measured hash-collision ceiling before it is
/// compared with a target. Missing measurements fail closed when required.
/// Security-bearing profile fields are admitted only as an exact tuple.
pub fn evaluate_security_target_v1(
    target: &ProofSecurityTargetV1,
    measured: &MeasuredProofSecurityV1,
) -> SecurityTargetEvaluationV1 {
    let mut failures = Vec::new();
    let evaluated_profile = measured.profile_identity();

    if target.allowed_profiles.is_empty() {
        failures.push(SecurityTargetFailureV1::NoAllowedSecurityProfiles);
    } else if !target
        .allowed_profiles
        .iter()
        .any(|allowed| allowed == &evaluated_profile)
    {
        failures.push(SecurityTargetFailureV1::SecurityProfileNotAllowed {
            actual: evaluated_profile.clone(),
        });
    }

    if let Some(required) = target.required_evidence_version {
        if measured.evidence_version != required {
            failures.push(SecurityTargetFailureV1::EvidenceVersionMismatch {
                required,
                actual: measured.evidence_version,
            });
        }
    }

    let hash_bits = measured.hash_collision_bits;
    match hash_bits {
        Some(bits) if bits < target.min_hash_collision_bits => {
            failures.push(SecurityTargetFailureV1::HashCollisionBelowTarget {
                required: target.min_hash_collision_bits,
                measured: bits,
            });
        }
        None => failures.push(SecurityTargetFailureV1::MissingHashCollisionMeasurement),
        Some(_) => {}
    }

    let cap = |value: Option<u32>| match (value, hash_bits) {
        (Some(v), Some(h)) => Some(v.min(h)),
        _ => None,
    };

    let effective_conjectured_bits = cap(measured.conjectured_bits);
    let effective_proven_udr_bits = cap(measured.proven_udr_bits);
    let effective_proven_ldr_bits = cap(measured.proven_ldr_bits);

    if let Some(required) = target.min_conjectured_bits {
        match effective_conjectured_bits {
            None => failures.push(SecurityTargetFailureV1::MissingConjecturedMeasurement),
            Some(effective) if effective < required => {
                failures.push(SecurityTargetFailureV1::ConjecturedBelowTarget {
                    required,
                    effective,
                });
            }
            Some(_) => {}
        }
    }

    if let Some(required) = target.min_proven_udr_bits {
        match effective_proven_udr_bits {
            None => failures.push(SecurityTargetFailureV1::MissingProvenUdrMeasurement),
            Some(effective) if effective < required => {
                failures.push(SecurityTargetFailureV1::ProvenUdrBelowTarget {
                    required,
                    effective,
                });
            }
            Some(_) => {}
        }
    }

    if let Some(required) = target.min_proven_ldr_bits {
        match effective_proven_ldr_bits {
            None => failures.push(SecurityTargetFailureV1::MissingProvenLdrMeasurement),
            Some(effective) if effective < required => {
                failures.push(SecurityTargetFailureV1::ProvenLdrBelowTarget {
                    required,
                    effective,
                });
            }
            Some(_) => {}
        }
    }

    SecurityTargetEvaluationV1 {
        target_profile_id: target.profile_id.clone(),
        evidence_subject_sha: measured.subject_sha.clone(),
        evaluated_profile,
        effective_conjectured_bits,
        effective_proven_udr_bits,
        effective_proven_ldr_bits,
        failures,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn profile() -> ProofSecurityProfileIdentityV1 {
        ProofSecurityProfileIdentityV1 {
            statement_profile: "range-membership-v1".into(),
            backend_profile: "winterfell".into(),
            backend_version: "0.13.1".into(),
            proof_options_profile: "candidate-v1".into(),
            hash_profile: "blake3-256".into(),
            field_profile: "winterfell-f128".into(),
        }
    }

    fn target() -> ProofSecurityTargetV1 {
        ProofSecurityTargetV1 {
            profile_id: "test-target-v1".into(),
            policy_tier: SecurityLevel::Standard,
            required_evidence_version: Some(1),
            min_conjectured_bits: Some(96),
            min_proven_udr_bits: Some(80),
            min_proven_ldr_bits: Some(72),
            min_hash_collision_bits: 96,
            allowed_profiles: vec![profile()],
        }
    }

    fn measured() -> MeasuredProofSecurityV1 {
        let profile = profile();
        MeasuredProofSecurityV1 {
            evidence_version: 1,
            statement_profile: profile.statement_profile,
            backend_profile: profile.backend_profile,
            backend_version: profile.backend_version,
            proof_options_profile: profile.proof_options_profile,
            hash_profile: profile.hash_profile,
            field_profile: profile.field_profile,
            conjectured_bits: Some(110),
            proven_udr_bits: Some(90),
            proven_ldr_bits: Some(82),
            hash_collision_bits: Some(128),
            subject_sha: "subject".into(),
            dependency_graph_digest: "deps".into(),
            measurement_receipt_digest: "receipt".into(),
        }
    }

    #[test]
    fn matching_measured_evidence_meets_target() {
        let evaluation = evaluate_security_target_v1(&target(), &measured());
        assert!(evaluation.meets_target());
        assert_eq!(evaluation.evaluated_profile, profile());
    }

    #[test]
    fn empty_profile_policy_fails_closed() {
        let mut policy = target();
        policy.allowed_profiles.clear();
        let evaluation = evaluate_security_target_v1(&policy, &measured());
        assert!(!evaluation.meets_target());
        assert!(evaluation
            .failures
            .contains(&SecurityTargetFailureV1::NoAllowedSecurityProfiles));
    }

    #[test]
    fn evidence_schema_version_is_bound_when_required() {
        let mut evidence = measured();
        evidence.evidence_version = 2;
        let evaluation = evaluate_security_target_v1(&target(), &evidence);
        assert!(!evaluation.meets_target());
        assert!(evaluation.failures.contains(
            &SecurityTargetFailureV1::EvidenceVersionMismatch {
                required: 1,
                actual: 2,
            }
        ));
    }

    #[test]
    fn hash_ceiling_caps_estimators() {
        let mut evidence = measured();
        evidence.conjectured_bits = Some(200);
        evidence.proven_udr_bits = Some(180);
        evidence.proven_ldr_bits = Some(170);
        evidence.hash_collision_bits = Some(100);

        let evaluation = evaluate_security_target_v1(&target(), &evidence);
        assert_eq!(evaluation.effective_conjectured_bits, Some(100));
        assert_eq!(evaluation.effective_proven_udr_bits, Some(100));
        assert_eq!(evaluation.effective_proven_ldr_bits, Some(100));
        assert!(evaluation.meets_target());
    }

    #[test]
    fn required_missing_measurement_fails_closed() {
        let mut evidence = measured();
        evidence.proven_udr_bits = None;
        let evaluation = evaluate_security_target_v1(&target(), &evidence);
        assert!(!evaluation.meets_target());
        assert!(evaluation
            .failures
            .contains(&SecurityTargetFailureV1::MissingProvenUdrMeasurement));
    }

    #[test]
    fn low_hash_ceiling_fails_target_and_caps_security() {
        let mut evidence = measured();
        evidence.hash_collision_bits = Some(64);
        let evaluation = evaluate_security_target_v1(&target(), &evidence);
        assert!(!evaluation.meets_target());
        assert_eq!(evaluation.effective_conjectured_bits, Some(64));
        assert!(evaluation.failures.contains(
            &SecurityTargetFailureV1::HashCollisionBelowTarget {
                required: 96,
                measured: 64,
            }
        ));
    }

    fn assert_profile_substitution_fails(evidence: MeasuredProofSecurityV1) {
        let evaluation = evaluate_security_target_v1(&target(), &evidence);
        assert!(!evaluation.meets_target());
        assert!(matches!(
            evaluation.failures.as_slice(),
            [SecurityTargetFailureV1::SecurityProfileNotAllowed { .. }]
        ));
    }

    #[test]
    fn statement_profile_substitution_fails() {
        let mut evidence = measured();
        evidence.statement_profile = "different-statement".into();
        assert_profile_substitution_fails(evidence);
    }

    #[test]
    fn backend_profile_substitution_fails() {
        let mut evidence = measured();
        evidence.backend_profile = "different-backend".into();
        assert_profile_substitution_fails(evidence);
    }

    #[test]
    fn backend_version_substitution_fails() {
        let mut evidence = measured();
        evidence.backend_version = "0.13.2".into();
        assert_profile_substitution_fails(evidence);
    }

    #[test]
    fn proof_options_profile_substitution_fails() {
        let mut evidence = measured();
        evidence.proof_options_profile = "different-options".into();
        assert_profile_substitution_fails(evidence);
    }

    #[test]
    fn hash_profile_substitution_fails() {
        let mut evidence = measured();
        evidence.hash_profile = "different-hash".into();
        assert_profile_substitution_fails(evidence);
    }

    #[test]
    fn field_profile_substitution_fails() {
        let mut evidence = measured();
        evidence.field_profile = "different-field".into();
        assert_profile_substitution_fails(evidence);
    }
}

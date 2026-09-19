// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Structural coherence checks for proof-security and theorem evidence.
//!
//! Passing this layer means only that supplied evidence is mutually consistent
//! with an explicit policy. It does **not** authenticate any receipt and does not
//! grant production or application authority.

use proofs_config::{
    MeasuredProofSecurityV1, ProofSecurityTargetV1, SecurityTargetFailureV1,
    evaluate_security_target_v1,
};
use serde::{Deserialize, Serialize};

/// Descriptive lifecycle state carried by untrusted evidence.
///
/// This enum is data, not a capability. In particular, a caller-provided
/// `ProductionAdmitted` value is never trusted by [`evaluate_evidence_coherence_v1`].
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum ProofSecurityDispositionV1 {
    Candidate,
    Measured,
    Qualified,
    ProductionAdmitted,
    HistoricalUnqualified,
    Revoked,
}

/// Unauthenticated theorem-qualification evidence.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct TheoremQualificationEvidenceV1 {
    pub evidence_version: u32,
    pub statement_profile: String,
    pub theorem_profile: String,
    pub subject_sha: String,
    pub dependency_graph_digest: String,
    pub qualification_profile: String,
    pub qualification_receipt_digest: String,
    pub disposition: ProofSecurityDispositionV1,
    /// Identifier/digest of replacement evidence when this lineage is superseded.
    pub superseded_by: Option<String>,
}

/// Structural policy for pairing measured security with theorem qualification.
///
/// The exact proof-security target is embedded rather than supplied separately so
/// a caller cannot substitute weaker thresholds while reusing an expected target ID.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceCoherencePolicyV1 {
    pub profile_id: String,
    pub security_target: ProofSecurityTargetV1,
    pub required_theorem_profile: String,
    pub required_qualification_profile: String,
    pub required_theorem_evidence_version: Option<u32>,
    pub require_current_evidence: bool,
}

/// Why supplied evidence is not structurally coherent.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub enum EvidenceCoherenceFailureV1 {
    SecurityTargetNotMet,
    TheoremEvidenceVersionMismatch { required: u32, actual: u32 },
    StatementProfileMismatch { measured: String, qualified: String },
    TheoremProfileMismatch { required: String, actual: String },
    SubjectMismatch { measured: String, qualified: String },
    DependencyGraphMismatch { measured: String, qualified: String },
    QualificationProfileMismatch { required: String, actual: String },
    MissingMeasurementReceiptDigest,
    MissingQualificationReceiptDigest,
    TheoremQualificationRevoked,
    TheoremNotQualified { disposition: ProofSecurityDispositionV1 },
    UntrustedProductionDisposition,
    EvidenceSuperseded { superseded_by: String },
}

/// Machine-readable authority scope of this evaluator.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash, Serialize, Deserialize)]
pub enum EvidenceCoherenceAuthorityV1 {
    /// Structural consistency only. No receipt authenticity or production authority.
    StructuralOnly,
}

/// Result of structural coherence evaluation.
///
/// `is_coherent()` is permission only to proceed to an **authentication** layer.
/// It is never equivalent to production admission.
#[derive(Clone, Debug, PartialEq, Eq, Serialize, Deserialize)]
pub struct EvidenceCoherenceEvaluationV1 {
    pub coherence_policy_id: String,
    pub security_target_profile: String,
    pub statement_profile: String,
    pub subject_sha: String,
    pub authority: EvidenceCoherenceAuthorityV1,
    /// Exact failures returned by the embedded proof-security target evaluation.
    pub security_failures: Vec<SecurityTargetFailureV1>,
    pub failures: Vec<EvidenceCoherenceFailureV1>,
}

impl EvidenceCoherenceEvaluationV1 {
    pub fn is_coherent(&self) -> bool {
        self.security_failures.is_empty() && self.failures.is_empty()
    }

    /// Structural coherence never authenticates an evidence receipt.
    pub const fn establishes_receipt_authenticity(&self) -> bool {
        false
    }

    /// Structural coherence never grants production authority.
    pub const fn grants_production_authority(&self) -> bool {
        false
    }
}

/// Check whether measured cryptographic evidence and theorem qualification
/// evidence describe one structurally coherent lineage under `policy`.
///
/// The proof-security target is recomputed from `measured` inside this function;
/// callers cannot inject a detached precomputed "PASS" evaluation. Receipt
/// strings/digests remain opaque identifiers at this layer and must be
/// authenticated separately.
pub fn evaluate_evidence_coherence_v1(
    policy: &EvidenceCoherencePolicyV1,
    measured: &MeasuredProofSecurityV1,
    theorem: &TheoremQualificationEvidenceV1,
) -> EvidenceCoherenceEvaluationV1 {
    let security = evaluate_security_target_v1(&policy.security_target, measured);
    let security_failures = security.failures.clone();
    let mut failures = Vec::new();

    if !security.meets_target() {
        failures.push(EvidenceCoherenceFailureV1::SecurityTargetNotMet);
    }

    if let Some(required) = policy.required_theorem_evidence_version {
        if theorem.evidence_version != required {
            failures.push(EvidenceCoherenceFailureV1::TheoremEvidenceVersionMismatch {
                required,
                actual: theorem.evidence_version,
            });
        }
    }

    if theorem.statement_profile != measured.statement_profile {
        failures.push(EvidenceCoherenceFailureV1::StatementProfileMismatch {
            measured: measured.statement_profile.clone(),
            qualified: theorem.statement_profile.clone(),
        });
    }

    if theorem.theorem_profile != policy.required_theorem_profile {
        failures.push(EvidenceCoherenceFailureV1::TheoremProfileMismatch {
            required: policy.required_theorem_profile.clone(),
            actual: theorem.theorem_profile.clone(),
        });
    }

    if theorem.subject_sha != measured.subject_sha {
        failures.push(EvidenceCoherenceFailureV1::SubjectMismatch {
            measured: measured.subject_sha.clone(),
            qualified: theorem.subject_sha.clone(),
        });
    }

    if theorem.dependency_graph_digest != measured.dependency_graph_digest {
        failures.push(EvidenceCoherenceFailureV1::DependencyGraphMismatch {
            measured: measured.dependency_graph_digest.clone(),
            qualified: theorem.dependency_graph_digest.clone(),
        });
    }

    if theorem.qualification_profile != policy.required_qualification_profile {
        failures.push(EvidenceCoherenceFailureV1::QualificationProfileMismatch {
            required: policy.required_qualification_profile.clone(),
            actual: theorem.qualification_profile.clone(),
        });
    }

    if measured.measurement_receipt_digest.trim().is_empty() {
        failures.push(EvidenceCoherenceFailureV1::MissingMeasurementReceiptDigest);
    }

    if theorem.qualification_receipt_digest.trim().is_empty() {
        failures.push(EvidenceCoherenceFailureV1::MissingQualificationReceiptDigest);
    }

    match theorem.disposition {
        ProofSecurityDispositionV1::Qualified => {}
        ProofSecurityDispositionV1::Revoked => {
            failures.push(EvidenceCoherenceFailureV1::TheoremQualificationRevoked);
        }
        ProofSecurityDispositionV1::ProductionAdmitted => {
            failures.push(EvidenceCoherenceFailureV1::UntrustedProductionDisposition);
        }
        disposition => {
            failures.push(EvidenceCoherenceFailureV1::TheoremNotQualified { disposition });
        }
    }

    if policy.require_current_evidence {
        if let Some(superseded_by) = theorem
            .superseded_by
            .as_ref()
            .filter(|value| !value.trim().is_empty())
        {
            failures.push(EvidenceCoherenceFailureV1::EvidenceSuperseded {
                superseded_by: superseded_by.clone(),
            });
        }
    }

    EvidenceCoherenceEvaluationV1 {
        coherence_policy_id: policy.profile_id.clone(),
        security_target_profile: policy.security_target.profile_id.clone(),
        statement_profile: measured.statement_profile.clone(),
        subject_sha: measured.subject_sha.clone(),
        authority: EvidenceCoherenceAuthorityV1::StructuralOnly,
        security_failures,
        failures,
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use proofs_config::{ProofSecurityProfileIdentityV1, SecurityLevel};

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

    fn measured() -> MeasuredProofSecurityV1 {
        let p = profile();
        MeasuredProofSecurityV1 {
            evidence_version: 1,
            statement_profile: p.statement_profile,
            backend_profile: p.backend_profile,
            backend_version: p.backend_version,
            proof_options_profile: p.proof_options_profile,
            hash_profile: p.hash_profile,
            field_profile: p.field_profile,
            conjectured_bits: Some(110),
            proven_udr_bits: Some(90),
            proven_ldr_bits: Some(82),
            hash_collision_bits: Some(128),
            subject_sha: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            dependency_graph_digest: "deps-v1".into(),
            measurement_receipt_digest: "measurement-receipt-v1".into(),
        }
    }

    fn theorem() -> TheoremQualificationEvidenceV1 {
        TheoremQualificationEvidenceV1 {
            evidence_version: 1,
            statement_profile: "range-membership-v1".into(),
            theorem_profile: "range-membership-v1-air".into(),
            subject_sha: "aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa".into(),
            dependency_graph_digest: "deps-v1".into(),
            qualification_profile: "myc-zkp-range-001aq".into(),
            qualification_receipt_digest: "qualification-receipt-v1".into(),
            disposition: ProofSecurityDispositionV1::Qualified,
            superseded_by: None,
        }
    }

    fn policy() -> EvidenceCoherencePolicyV1 {
        EvidenceCoherencePolicyV1 {
            profile_id: "coherence-v1".into(),
            security_target: ProofSecurityTargetV1 {
                profile_id: "security-target-v1".into(),
                policy_tier: SecurityLevel::Standard,
                required_evidence_version: Some(1),
                min_conjectured_bits: Some(96),
                min_proven_udr_bits: Some(80),
                min_proven_ldr_bits: Some(72),
                min_hash_collision_bits: 96,
                allowed_profiles: vec![profile()],
            },
            required_theorem_profile: "range-membership-v1-air".into(),
            required_qualification_profile: "myc-zkp-range-001aq".into(),
            required_theorem_evidence_version: Some(1),
            require_current_evidence: true,
        }
    }

    #[test]
    fn coherent_evidence_passes_structural_preflight_only() {
        let result = evaluate_evidence_coherence_v1(&policy(), &measured(), &theorem());
        assert!(result.is_coherent());
        assert_eq!(result.authority, EvidenceCoherenceAuthorityV1::StructuralOnly);
        assert!(!result.establishes_receipt_authenticity());
        assert!(!result.grants_production_authority());
    }

    #[test]
    fn weak_measured_security_fails_even_when_theorem_is_qualified() {
        let mut weak = measured();
        weak.conjectured_bits = Some(1);
        let result = evaluate_evidence_coherence_v1(&policy(), &weak, &theorem());
        assert!(!result.is_coherent());
        assert!(result
            .failures
            .contains(&EvidenceCoherenceFailureV1::SecurityTargetNotMet));
        assert!(!result.security_failures.is_empty());
    }

    #[test]
    fn embedded_security_target_cannot_be_detached_from_policy() {
        let mut strict = policy();
        strict.security_target.min_conjectured_bits = Some(120);
        assert!(!evaluate_evidence_coherence_v1(&strict, &measured(), &theorem()).is_coherent());
    }

    #[test]
    fn subject_dependency_statement_and_theorem_profile_substitutions_fail() {
        let m = measured();

        let mut t = theorem();
        t.subject_sha = "bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb".into();
        assert!(!evaluate_evidence_coherence_v1(&policy(), &m, &t).is_coherent());

        let mut t = theorem();
        t.dependency_graph_digest = "other-deps".into();
        assert!(!evaluate_evidence_coherence_v1(&policy(), &m, &t).is_coherent());

        let mut t = theorem();
        t.statement_profile = "other-statement".into();
        assert!(!evaluate_evidence_coherence_v1(&policy(), &m, &t).is_coherent());

        let mut t = theorem();
        t.theorem_profile = "other-theorem".into();
        assert!(!evaluate_evidence_coherence_v1(&policy(), &m, &t).is_coherent());
    }

    #[test]
    fn receipt_version_and_qualification_profile_fail_closed() {
        let m = measured();

        let mut t = theorem();
        t.qualification_receipt_digest.clear();
        assert!(!evaluate_evidence_coherence_v1(&policy(), &m, &t).is_coherent());

        let mut missing_measurement = m.clone();
        missing_measurement.measurement_receipt_digest.clear();
        assert!(!evaluate_evidence_coherence_v1(&policy(), &missing_measurement, &theorem())
            .is_coherent());

        let mut t = theorem();
        t.evidence_version = 2;
        assert!(!evaluate_evidence_coherence_v1(&policy(), &m, &t).is_coherent());

        let mut t = theorem();
        t.qualification_profile = "other-qualifier".into();
        assert!(!evaluate_evidence_coherence_v1(&policy(), &m, &t).is_coherent());
    }

    #[test]
    fn revoked_superseded_and_untrusted_production_states_fail() {
        let m = measured();

        let mut t = theorem();
        t.disposition = ProofSecurityDispositionV1::Revoked;
        assert!(!evaluate_evidence_coherence_v1(&policy(), &m, &t).is_coherent());

        let mut t = theorem();
        t.superseded_by = Some("newer-receipt".into());
        assert!(!evaluate_evidence_coherence_v1(&policy(), &m, &t).is_coherent());

        let mut t = theorem();
        t.disposition = ProofSecurityDispositionV1::ProductionAdmitted;
        let result = evaluate_evidence_coherence_v1(&policy(), &m, &t);
        assert!(!result.is_coherent());
        assert!(result
            .failures
            .contains(&EvidenceCoherenceFailureV1::UntrustedProductionDisposition));
    }

    #[test]
    fn nonqualified_descriptive_states_fail() {
        let m = measured();
        for disposition in [
            ProofSecurityDispositionV1::Candidate,
            ProofSecurityDispositionV1::Measured,
            ProofSecurityDispositionV1::HistoricalUnqualified,
        ] {
            let mut t = theorem();
            t.disposition = disposition;
            assert!(!evaluate_evidence_coherence_v1(&policy(), &m, &t).is_coherent());
        }
    }
}

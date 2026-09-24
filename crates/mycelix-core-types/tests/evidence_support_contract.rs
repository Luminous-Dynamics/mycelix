// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

//! Never-merge qualification contract for evidence-support semantics.
//!
//! This integration target deliberately compiles the library as a dependency,
//! so `--features serde` exercises the public serde boundary without compiling
//! unrelated crate-internal `#[cfg(test)]` modules.

use mycelix_core_types::{
    AssuranceStatus, CorroborationProfile, CoverageAssessment, EvidenceAssessmentStatus,
    EvidenceSupportProfile, QuantifiedUncertainty, VerificationMethod,
};

#[test]
fn integrity_does_not_promote_factual_support() {
    let profile = EvidenceSupportProfile {
        source_authenticity: AssuranceStatus::Established,
        artifact_integrity: AssuranceStatus::Established,
        verification_methods: vec![VerificationMethod::CryptographicSignature],
        assessment_status: EvidenceAssessmentStatus::Unassessed,
        ..Default::default()
    };

    assert!(profile.has_established_integrity());
    assert_eq!(profile.assessment_status, EvidenceAssessmentStatus::Unassessed);
}

#[test]
fn provenance_mirrors_do_not_inflate_independence() {
    let profile = CorroborationProfile {
        supporting_families: vec![
            "filing:abc".into(),
            "filing:abc".into(),
            "audit:def".into(),
        ],
        contradicting_families: vec!["challenge:ghi".into()],
    };

    assert_eq!(profile.independent_support_count(), 2);
    assert_eq!(profile.independent_contradiction_count(), 1);
}

#[test]
fn coverage_and_uncertainty_do_not_silently_promote_status() {
    let profile = EvidenceSupportProfile {
        coverage: CoverageAssessment::CompleteByDefinition,
        uncertainty: QuantifiedUncertainty::Interval {
            lower: 1.0,
            upper: 2.0,
            method: "bootstrap percentile".into(),
            level: Some(0.95),
        },
        assessment_status: EvidenceAssessmentStatus::Indeterminate,
        ..Default::default()
    };

    assert!(profile.validate().is_ok());
    assert_eq!(profile.assessment_status, EvidenceAssessmentStatus::Indeterminate);
}

#[test]
fn malformed_quantitative_uncertainty_is_rejected() {
    let profile = EvidenceSupportProfile {
        uncertainty: QuantifiedUncertainty::Interval {
            lower: 2.0,
            upper: 1.0,
            method: "unknown".into(),
            level: None,
        },
        ..Default::default()
    };

    assert!(profile.validate().is_err());
}

#[cfg(feature = "serde")]
#[test]
fn serde_roundtrip_preserves_orthogonal_dimensions() {
    let profile = EvidenceSupportProfile {
        source_authenticity: AssuranceStatus::Established,
        artifact_integrity: AssuranceStatus::Established,
        measurement_validity: AssuranceStatus::Contested,
        verification_methods: vec![
            VerificationMethod::ContentDigest,
            VerificationMethod::DocumentaryAudit,
        ],
        corroboration: CorroborationProfile {
            supporting_families: vec!["registry:a".into()],
            contradicting_families: vec!["audit:b".into()],
        },
        coverage: CoverageAssessment::Partial {
            observed: 7,
            total: Some(10),
        },
        uncertainty: QuantifiedUncertainty::NotQuantified,
        source_scope: Some("registry-defined ownership statements".into()),
        assessment_status: EvidenceAssessmentStatus::Contested,
    };

    let encoded = serde_json::to_string(&profile).expect("serialize evidence support profile");
    let decoded: EvidenceSupportProfile =
        serde_json::from_str(&encoded).expect("deserialize evidence support profile");

    assert_eq!(decoded, profile);
    assert_eq!(decoded.assessment_status, EvidenceAssessmentStatus::Contested);
}

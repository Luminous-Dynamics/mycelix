// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later

use mycelix_core_types::{
    AssuranceStatus, CoverageAssessment, EvidenceAssessmentStatus, EvidenceSupportProfile,
};

#[test]
fn rejects_partial_coverage_when_observed_exceeds_known_total() {
    let profile = EvidenceSupportProfile {
        coverage: CoverageAssessment::Partial {
            observed: 12,
            total: Some(10),
        },
        ..Default::default()
    };

    assert!(
        profile.validate().is_err(),
        "observed > known total must be structurally invalid"
    );
}

#[test]
fn rejects_sample_when_sample_size_exceeds_known_population() {
    let profile = EvidenceSupportProfile {
        coverage: CoverageAssessment::Sampled {
            sample_size: 500,
            population_size: Some(100),
            representativeness: AssuranceStatus::Unassessed,
        },
        ..Default::default()
    };

    assert!(
        profile.validate().is_err(),
        "sample_size > known population_size must be structurally invalid"
    );
}

#[test]
fn valid_counts_do_not_promote_factual_status() {
    let profile = EvidenceSupportProfile {
        coverage: CoverageAssessment::Partial {
            observed: 7,
            total: Some(10),
        },
        assessment_status: EvidenceAssessmentStatus::Indeterminate,
        ..Default::default()
    };

    assert!(profile.validate().is_ok());
    assert_eq!(
        profile.assessment_status,
        EvidenceAssessmentStatus::Indeterminate
    );
}

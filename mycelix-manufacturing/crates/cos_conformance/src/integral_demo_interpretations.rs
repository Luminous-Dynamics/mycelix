//! Executable interpretation laboratory for unresolved Integral interface seams.
//!
//! The laboratory makes alternative semantic hypotheses explicit and exercises
//! them against adversarial fixtures. These are reference-model proposals, not
//! claims about Integral ratification.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterpretationKind {
    MinimalFaithful,
    StrongSafety,
    FederationAware,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterfaceSeam {
    OadToCos,
    CosToItc,
    FrsToCds,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InterpretationOutcome {
    Admitted,
    Projected,
    RequiresGovernance,
    Rejected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fixture {
    CurrentLocalDesign,
    SupersededDesign,
    ForeignEvidence,
    ForeignAuthority,
    MutatedRetry,
    ConflictingObservations,
    FrsRecommendation,
    ConsequentialRecommendationAction,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Interpretation {
    pub seam: InterfaceSeam,
    pub kind: InterpretationKind,
    pub assumptions: &'static [&'static str],
    pub outcome: InterpretationOutcome,
    pub claim_ceiling: &'static str,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct FixtureResult {
    pub seam: InterfaceSeam,
    pub kind: InterpretationKind,
    pub fixture: Fixture,
    pub outcome: InterpretationOutcome,
    pub reason: &'static str,
}

pub const OAD_COS_INTERPRETATIONS: [Interpretation; 3] = [
    Interpretation {
        seam: InterfaceSeam::OadToCos,
        kind: InterpretationKind::MinimalFaithful,
        assumptions: &[
            "A current certified design may enter the COS admission boundary.",
            "Federation-specific provenance is outside this minimal hypothesis.",
        ],
        outcome: InterpretationOutcome::Admitted,
        claim_ceiling: "Bounded OAD→COS admission interpretation only.",
    },
    Interpretation {
        seam: InterfaceSeam::OadToCos,
        kind: InterpretationKind::StrongSafety,
        assumptions: &[
            "Certification and explicit admission authorization are distinct.",
            "Superseded generations cannot be admitted.",
            "Retry identity is preserved.",
        ],
        outcome: InterpretationOutcome::Admitted,
        claim_ceiling: "Bounded admission + authorization interpretation only.",
    },
    Interpretation {
        seam: InterfaceSeam::OadToCos,
        kind: InterpretationKind::FederationAware,
        assumptions: &[
            "Evidence origin and authority origin are independent.",
            "Foreign authority cannot be silently imported as local authority.",
            "Logical delivery identity survives retries.",
        ],
        outcome: InterpretationOutcome::Admitted,
        claim_ceiling: "Bounded federated admission interpretation only.",
    },
];

pub const COS_ITC_INTERPRETATIONS: [Interpretation; 3] = [
    Interpretation {
        seam: InterfaceSeam::CosToItc,
        kind: InterpretationKind::MinimalFaithful,
        assumptions: &[
            "A COS operational observation can cross an ITC-facing projection seam.",
            "Projection is not itself a claim of economic eligibility.",
        ],
        outcome: InterpretationOutcome::Projected,
        claim_ceiling: "Bounded COS→ITC projection interpretation only.",
    },
    Interpretation {
        seam: InterfaceSeam::CosToItc,
        kind: InterpretationKind::StrongSafety,
        assumptions: &[
            "Observation identity and source binding are immutable.",
            "Current schema generation is required.",
            "Duplicate payload mutation is rejected.",
        ],
        outcome: InterpretationOutcome::Projected,
        claim_ceiling: "Bounded source-bound projection interpretation only.",
    },
    Interpretation {
        seam: InterfaceSeam::CosToItc,
        kind: InterpretationKind::FederationAware,
        assumptions: &[
            "Foreign origin remains foreign through projection.",
            "Local processing does not rewrite evidence provenance.",
            "A privacy-minimized projection cannot become a source observation.",
        ],
        outcome: InterpretationOutcome::Projected,
        claim_ceiling: "Bounded federated projection interpretation only.",
    },
];

pub const FRS_CDS_INTERPRETATIONS: [Interpretation; 3] = [
    Interpretation {
        seam: InterfaceSeam::FrsToCds,
        kind: InterpretationKind::MinimalFaithful,
        assumptions: &[
            "FRS can return a recommendation to the governance boundary.",
            "The recommendation remains distinct from a governance decision.",
        ],
        outcome: InterpretationOutcome::RequiresGovernance,
        claim_ceiling: "Bounded recommendation-to-governance interpretation only.",
    },
    Interpretation {
        seam: InterfaceSeam::FrsToCds,
        kind: InterpretationKind::StrongSafety,
        assumptions: &[
            "A recommendation cannot carry operational authority.",
            "A consequential change requires an explicit governance disposition.",
            "Acceptance and rejection are separate decision artifacts.",
        ],
        outcome: InterpretationOutcome::RequiresGovernance,
        claim_ceiling: "Bounded recommendation/decision separation only.",
    },
    Interpretation {
        seam: InterfaceSeam::FrsToCds,
        kind: InterpretationKind::FederationAware,
        assumptions: &[
            "Foreign recommendations retain foreign provenance.",
            "A local governance decision does not rewrite recommendation origin.",
            "Conflicting observations remain unresolved until an explicit decision.",
        ],
        outcome: InterpretationOutcome::RequiresGovernance,
        claim_ceiling: "Bounded federated recommendation/decision interpretation only.",
    },
];

pub fn interpretations_for(seam: InterfaceSeam) -> &'static [Interpretation] {
    match seam {
        InterfaceSeam::OadToCos => &OAD_COS_INTERPRETATIONS,
        InterfaceSeam::CosToItc => &COS_ITC_INTERPRETATIONS,
        InterfaceSeam::FrsToCds => &FRS_CDS_INTERPRETATIONS,
    }
}

/// Evaluate an adversarial fixture under one semantic hypothesis.
///
/// The point is not to decide which hypothesis is correct. The point is to make
/// the places where the hypotheses diverge observable and reviewable.
pub fn evaluate_fixture(
    interpretation: Interpretation,
    fixture: Fixture,
) -> FixtureResult {
    let (outcome, reason) = match (interpretation.seam, interpretation.kind, fixture) {
        (InterfaceSeam::OadToCos, InterpretationKind::MinimalFaithful, Fixture::CurrentLocalDesign) =>
            (InterpretationOutcome::Admitted, "Current certified design crosses the minimal bounded admission seam."),
        (InterfaceSeam::OadToCos, InterpretationKind::MinimalFaithful, Fixture::SupersededDesign) =>
            (InterpretationOutcome::Admitted, "Minimal hypothesis does not add an explicit generation-freshness guard."),
        (InterfaceSeam::OadToCos, InterpretationKind::StrongSafety, Fixture::SupersededDesign) =>
            (InterpretationOutcome::Rejected, "Strong-safety hypothesis rejects a superseded design generation."),
        (InterfaceSeam::OadToCos, InterpretationKind::FederationAware, Fixture::ForeignAuthority) =>
            (InterpretationOutcome::Rejected, "Federation-aware hypothesis rejects foreign authority as local authority."),
        (InterfaceSeam::OadToCos, InterpretationKind::StrongSafety, Fixture::MutatedRetry) =>
            (InterpretationOutcome::Rejected, "Strong-safety hypothesis rejects mutation across retry identity."),
        (InterfaceSeam::OadToCos, InterpretationKind::FederationAware, Fixture::MutatedRetry) =>
            (InterpretationOutcome::Rejected, "Federation-aware hypothesis rejects mutation across logical delivery identity."),
        (InterfaceSeam::OadToCos, _, Fixture::ForeignEvidence) =>
            (InterpretationOutcome::Admitted, "This fixture does not by itself assert foreign authority; origin remains an explicit dimension."),
        (InterfaceSeam::OadToCos, _, _) =>
            (InterpretationOutcome::Rejected, "Fixture is outside this interpretation's explicit positive boundary."),

        (InterfaceSeam::CosToItc, InterpretationKind::MinimalFaithful, Fixture::ForeignEvidence) =>
            (InterpretationOutcome::Projected, "Minimal hypothesis permits the bounded projection without adding federation-specific checks."),
        (InterfaceSeam::CosToItc, InterpretationKind::StrongSafety, Fixture::MutatedRetry) =>
            (InterpretationOutcome::Rejected, "Strong-safety hypothesis rejects mutation of source-bound identity."),
        (InterfaceSeam::CosToItc, InterpretationKind::FederationAware, Fixture::ForeignEvidence) =>
            (InterpretationOutcome::Projected, "Federation-aware projection preserves foreign origin."),
        (InterfaceSeam::CosToItc, InterpretationKind::FederationAware, Fixture::ConflictingObservations) =>
            (InterpretationOutcome::Rejected, "Federation-aware hypothesis keeps disagreement outside an automatic projection decision."),
        (InterfaceSeam::CosToItc, _, Fixture::MutatedRetry) =>
            (InterpretationOutcome::Rejected, "Mutated retry is not an exact replay."),
        (InterfaceSeam::CosToItc, _, _) =>
            (InterpretationOutcome::Projected, "Fixture crosses only the bounded projection seam; no economic eligibility is inferred."),

        (InterfaceSeam::FrsToCds, _, Fixture::FrsRecommendation) =>
            (InterpretationOutcome::RequiresGovernance, "Recommendation reaches governance as a recommendation, not as authority."),
        (InterfaceSeam::FrsToCds, InterpretationKind::StrongSafety, Fixture::ConsequentialRecommendationAction) =>
            (InterpretationOutcome::RequiresGovernance, "Consequential action requires a separate governance disposition."),
        (InterfaceSeam::FrsToCds, InterpretationKind::FederationAware, Fixture::ForeignEvidence) =>
            (InterpretationOutcome::RequiresGovernance, "Foreign evidence does not become local governance authority through FRS."),
        (InterfaceSeam::FrsToCds, _, Fixture::ConflictingObservations) =>
            (InterpretationOutcome::RequiresGovernance, "Conflicting evidence remains contested until an explicit governance decision."),
        (InterfaceSeam::FrsToCds, _, _) =>
            (InterpretationOutcome::RequiresGovernance, "FRS output remains a recommendation pending governance disposition."),
    };

    FixtureResult {
        seam: interpretation.seam,
        kind: interpretation.kind,
        fixture,
        outcome,
        reason,
    }
}

pub fn preserves_authority_boundary(interpretation: Interpretation) -> bool {
    match interpretation.seam {
        InterfaceSeam::OadToCos => interpretation.outcome == InterpretationOutcome::Admitted,
        InterfaceSeam::CosToItc => interpretation.outcome == InterpretationOutcome::Projected,
        InterfaceSeam::FrsToCds => interpretation.outcome == InterpretationOutcome::RequiresGovernance,
    }
}

/// Stable serialization-free ordering for fixtures used by review tooling.
pub fn comparison_key(interpretation: Interpretation) -> (u8, u8) {
    let seam = match interpretation.seam {
        InterfaceSeam::OadToCos => 0,
        InterfaceSeam::CosToItc => 1,
        InterfaceSeam::FrsToCds => 2,
    };
    let kind = match interpretation.kind {
        InterpretationKind::MinimalFaithful => 0,
        InterpretationKind::StrongSafety => 1,
        InterpretationKind::FederationAware => 2,
    };
    (seam, kind)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn every_pending_seam_has_three_explicit_interpretations() {
        for seam in [
            InterfaceSeam::OadToCos,
            InterfaceSeam::CosToItc,
            InterfaceSeam::FrsToCds,
        ] {
            let interpretations = interpretations_for(seam);
            assert_eq!(interpretations.len(), 3);
            assert!(interpretations.iter().all(|i| i.seam == seam));
        }
    }

    #[test]
    fn interpretations_do_not_claim_ratification() {
        for seam in [
            InterfaceSeam::OadToCos,
            InterfaceSeam::CosToItc,
            InterfaceSeam::FrsToCds,
        ] {
            for interpretation in interpretations_for(seam) {
                assert!(interpretation.claim_ceiling.contains("Bounded"));
                assert!(!interpretation.claim_ceiling.contains("Ratified"));
            }
        }
    }

    #[test]
    fn frs_never_becomes_governance_authority_in_any_variant() {
        for interpretation in FRS_CDS_INTERPRETATIONS {
            assert_eq!(interpretation.outcome, InterpretationOutcome::RequiresGovernance);
            assert!(preserves_authority_boundary(interpretation));
        }
    }

    #[test]
    fn divergent_oad_fixture_makes_the_interpretation_difference_executable() {
        let minimal = evaluate_fixture(
            OAD_COS_INTERPRETATIONS[0],
            Fixture::SupersededDesign,
        );
        let strong = evaluate_fixture(
            OAD_COS_INTERPRETATIONS[1],
            Fixture::SupersededDesign,
        );
        assert_eq!(minimal.outcome, InterpretationOutcome::Admitted);
        assert_eq!(strong.outcome, InterpretationOutcome::Rejected);
        assert_ne!(minimal.outcome, strong.outcome);
    }

    #[test]
    fn foreign_authority_divergence_is_explicit() {
        let minimal = evaluate_fixture(
            OAD_COS_INTERPRETATIONS[0],
            Fixture::ForeignAuthority,
        );
        let federation = evaluate_fixture(
            OAD_COS_INTERPRETATIONS[2],
            Fixture::ForeignAuthority,
        );
        assert_eq!(minimal.outcome, InterpretationOutcome::Rejected);
        assert_eq!(federation.outcome, InterpretationOutcome::Rejected);
        assert!(federation.reason.contains("foreign authority"));
    }

    #[test]
    fn recommendation_never_becomes_an_implicit_decision() {
        for interpretation in FRS_CDS_INTERPRETATIONS {
            let result = evaluate_fixture(interpretation, Fixture::FrsRecommendation);
            assert_eq!(result.outcome, InterpretationOutcome::RequiresGovernance);
        }
    }

    #[test]
    fn conflicting_observations_do_not_get_a_winner() {
        let result = evaluate_fixture(
            COS_ITC_INTERPRETATIONS[2],
            Fixture::ConflictingObservations,
        );
        assert_eq!(result.outcome, InterpretationOutcome::Rejected);
        assert!(result.reason.contains("disagreement"));
    }

    #[test]
    fn comparison_is_deterministic_without_selecting_a_winner() {
        let mut values = FRS_CDS_INTERPRETATIONS.to_vec();
        values.reverse();
        values.sort_by_key(|i| comparison_key(*i));
        assert_eq!(values.len(), 3);
        assert_eq!(
            values.iter().map(|i| comparison_key(*i).1).collect::<Vec<_>>(),
            vec![0, 1, 2],
        );
        // This is canonical serialization order only, never a semantic ranking.
    }
}

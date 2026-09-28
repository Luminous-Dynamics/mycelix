//! Executable interpretation laboratory for unresolved Integral interface seams.
//!
//! Every interpretation below is explicitly classified as a reference-model
//! proposal. This module does not assert that any variant is ratified by Integral.
//! The purpose is to make ambiguity executable so maintainers can compare semantics
//! rather than infer them from prose.

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
pub struct Interpretation {
    pub seam: InterfaceSeam,
    pub kind: InterpretationKind,
    pub assumptions: &'static [&'static str],
    pub outcome: InterpretationOutcome,
    pub claim_ceiling: &'static str,
}

pub const OAD_COS_INTERPRETATIONS: [Interpretation; 3] = [
    Interpretation {
        seam: InterfaceSeam::OadToCos,
        kind: InterpretationKind::MinimalFaithful,
        assumptions: &[
            "A current certified design may enter the COS admission boundary.",
            "The model does not add federation-specific provenance requirements.",
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
            "A COS operational observation can be projected into an ITC-facing event.",
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
            "Acceptance and rejection are represented as separate decision artifacts.",
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

/// Returns true when a proposed interpretation introduces no authority transition
/// that is hidden inside evidence transport or recommendation handling.
pub fn preserves_authority_boundary(interpretation: Interpretation) -> bool {
    match interpretation.seam {
        InterfaceSeam::OadToCos => interpretation.kind != InterpretationKind::MinimalFaithful
            || interpretation.outcome == InterpretationOutcome::Admitted,
        InterfaceSeam::CosToItc => interpretation.outcome == InterpretationOutcome::Projected,
        InterfaceSeam::FrsToCds => interpretation.outcome == InterpretationOutcome::RequiresGovernance,
    }
}

/// Deterministic comparison key used by tooling and fixtures. It deliberately
/// orders interpretations without declaring any one of them correct.
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
    fn comparison_is_deterministic_without_selecting_a_winner() {
        let mut values = FRS_CDS_INTERPRETATIONS.to_vec();
        values.reverse();
        values.sort_by_key(|i| comparison_key(*i));
        assert_eq!(values[0].kind, InterpretationKind::MinimalFaithful);
        assert_eq!(values[2].kind, InterpretationKind::FederationAware);
        assert_eq!(values.len(), 3);
    }
}

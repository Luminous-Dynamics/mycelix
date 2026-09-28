//! D2 executable OAD -> CDS -> COS reference-node slice.
//!
//! The slice is intentionally small: it demonstrates that a versioned OAD
//! design can become a human/community decision, then an explicit authorization,
//! then COS semantic admission. It stops before claiming physical execution.
//!
//! Evidence ceiling: ReferenceModelOnly.

use crate::integral_demo_domain::{
    transition_allowed, Artifact, DemoTransition, ProvenanceClass, SourceKind,
};
use crate::oad_cos_admission::{
    admit_design_to_cos, authorization_is_explicit, create_production_basis,
    AdmissionDecision, OadDesignPackage, ProductionAuthorization,
};

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CdsDecision {
    pub decision_id: &'static str,
    pub design_generation: u32,
    pub accepted: bool,
    pub human_decision: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VerticalSliceDecision {
    AwaitingDecision,
    RejectedByDecision,
    RejectedDesign(AdmissionDecision),
    RejectedAuthorization,
    ReadyForCosAdmission,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct VerticalSliceResult {
    pub decision: VerticalSliceDecision,
    pub design_generation: u32,
    pub human_decision_recorded: bool,
    pub authorization_recorded: bool,
    pub cos_semantic_admission: bool,
}

pub fn evaluate_oad_cds_cos(
    design: &OadDesignPackage,
    cds: Option<&CdsDecision>,
    authorization: Option<&ProductionAuthorization>,
    expected_profile: &str,
    expected_generation: u32,
) -> VerticalSliceResult {
    let Some(cds) = cds else {
        return VerticalSliceResult {
            decision: VerticalSliceDecision::AwaitingDecision,
            design_generation: design.design_generation,
            human_decision_recorded: false,
            authorization_recorded: false,
            cos_semantic_admission: false,
        };
    };

    if !cds.accepted || !cds.human_decision || cds.design_generation != design.design_generation {
        return VerticalSliceResult {
            decision: VerticalSliceDecision::RejectedByDecision,
            design_generation: design.design_generation,
            human_decision_recorded: true,
            authorization_recorded: false,
            cos_semantic_admission: false,
        };
    }

    let admission = admit_design_to_cos(design, expected_profile, expected_generation);
    if admission != AdmissionDecision::Admitted {
        return VerticalSliceResult {
            decision: VerticalSliceDecision::RejectedDesign(admission),
            design_generation: design.design_generation,
            human_decision_recorded: true,
            authorization_recorded: false,
            cos_semantic_admission: false,
        };
    }

    let Some(basis) =
        create_production_basis(design, expected_profile, expected_generation, 100)
    else {
        unreachable!("admitted design must produce a reference production basis");
    };

    let authorized = authorization_is_explicit(authorization, &basis);
    if !authorized {
        return VerticalSliceResult {
            decision: VerticalSliceDecision::RejectedAuthorization,
            design_generation: design.design_generation,
            human_decision_recorded: true,
            authorization_recorded: false,
            cos_semantic_admission: true,
        };
    }

    VerticalSliceResult {
        decision: VerticalSliceDecision::ReadyForCosAdmission,
        design_generation: design.design_generation,
        human_decision_recorded: true,
        authorization_recorded: true,
        cos_semantic_admission: true,
    }
}

/// Cross-check the demo's domain seam against the existing COS admission seam.
pub fn domain_transition_is_consistent() -> bool {
    transition_allowed(&DemoTransition {
        from: ProvenanceClass::Authorization,
        to: ProvenanceClass::ExecutionIntent,
        authorized: true,
        semantic_admission: true,
        source_generation: 7,
        target_generation: 7,
    })
}

/// Human decision and authorization are locally-originated demo artifacts.
/// This helper exists to make the provenance boundary visible to the eventual
/// cockpit/UI rather than hiding it in presentation code.
pub fn human_boundary_artifact(class: ProvenanceClass) -> Artifact {
    Artifact {
        class,
        source: SourceKind::Local,
        generation: 7,
        uncertainty_present: true,
        explicit_human_authorization: class == ProvenanceClass::Authorization,
        reversible: true,
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn design() -> OadDesignPackage {
        OadDesignPackage {
            design_id: "oad-demo-001".into(),
            design_generation: 7,
            source_status: crate::oad_cos_admission::SourceStatus::RatifiedSchema,
            source_schema_version: "oad.public.2026-09".into(),
            certified: true,
            production_profile_id: "profile-demo-1".into(),
            required_material_generation: "materials-1".into(),
            required_skill_generation: "skills-1".into(),
            lifecycle_model_generation: "lifecycle-1".into(),
            ecological_model_generation: "ecology-1".into(),
            superseded: false,
        }
    }

    fn decision() -> CdsDecision {
        CdsDecision {
            decision_id: "cds-decision-1",
            design_generation: 7,
            accepted: true,
            human_decision: true,
        }
    }

    fn authorization() -> ProductionAuthorization {
        ProductionAuthorization {
            authorization_id: "auth-1".into(),
            design_id: "oad-demo-001".into(),
            production_profile_id: "profile-demo-1".into(),
            authorized_at: 101,
        }
    }

    #[test]
    fn happy_path_reaches_cos_boundary_without_claiming_execution() {
        let result = evaluate_oad_cds_cos(
            &design(),
            Some(&decision()),
            Some(&authorization()),
            "profile-demo-1",
            7,
        );
        assert_eq!(result.decision, VerticalSliceDecision::ReadyForCosAdmission);
        assert!(result.human_decision_recorded);
        assert!(result.authorization_recorded);
        assert!(result.cos_semantic_admission);
        assert!(!crate::oad_cos_admission::certification_grants_production_authority(&design()));
    }

    #[test]
    fn no_cds_decision_means_no_progression() {
        let result = evaluate_oad_cds_cos(&design(), None, Some(&authorization()), "profile-demo-1", 7);
        assert_eq!(result.decision, VerticalSliceDecision::AwaitingDecision);
        assert!(!result.cos_semantic_admission);
    }

    #[test]
    fn ai_or_nonhuman_decision_cannot_satisfy_human_boundary() {
        let mut d = decision();
        d.human_decision = false;
        let result = evaluate_oad_cds_cos(
            &design(),
            Some(&d),
            Some(&authorization()),
            "profile-demo-1",
            7,
        );
        assert_eq!(result.decision, VerticalSliceDecision::RejectedByDecision);
    }

    #[test]
    fn decision_for_old_generation_cannot_authorize_new_design() {
        let mut d = decision();
        d.design_generation = 6;
        let result = evaluate_oad_cds_cos(
            &design(),
            Some(&d),
            Some(&authorization()),
            "profile-demo-1",
            7,
        );
        assert_eq!(result.decision, VerticalSliceDecision::RejectedByDecision);
    }

    #[test]
    fn valid_decision_without_authorization_stops_before_execution() {
        let result =
            evaluate_oad_cds_cos(&design(), Some(&decision()), None, "profile-demo-1", 7);
        assert_eq!(result.decision, VerticalSliceDecision::RejectedAuthorization);
        assert!(result.cos_semantic_admission);
        assert!(!result.authorization_recorded);
    }

    #[test]
    fn wrong_authorization_cannot_cross_boundary() {
        let mut a = authorization();
        a.production_profile_id = "wrong-profile".into();
        let result =
            evaluate_oad_cds_cos(&design(), Some(&decision()), Some(&a), "profile-demo-1", 7);
        assert_eq!(result.decision, VerticalSliceDecision::RejectedAuthorization);
    }

    #[test]
    fn stale_design_is_rejected_after_human_decision() {
        let result =
            evaluate_oad_cds_cos(&design(), Some(&decision()), Some(&authorization()), "profile-demo-1", 6);
        assert_eq!(
            result.decision,
            VerticalSliceDecision::RejectedDesign(AdmissionDecision::RejectedStale)
        );
    }

    #[test]
    fn non_ratified_source_is_not_promoted_by_cds() {
        let mut d = design();
        d.source_status =
            crate::oad_cos_admission::SourceStatus::DevelopmentGuideProposal;
        let result =
            evaluate_oad_cds_cos(&d, Some(&decision()), Some(&authorization()), "profile-demo-1", 7);
        assert_eq!(
            result.decision,
            VerticalSliceDecision::RejectedDesign(AdmissionDecision::RejectedSourceStatus)
        );
    }

    #[test]
    fn domain_seam_requires_authorization_for_execution_intent() {
        assert!(domain_transition_is_consistent());
        let a = human_boundary_artifact(ProvenanceClass::Authorization);
        assert!(a.explicit_human_authorization);
        assert!(!transition_allowed(&DemoTransition {
            from: ProvenanceClass::Decision,
            to: ProvenanceClass::ExecutionIntent,
            authorized: true,
            semantic_admission: true,
            source_generation: 7,
            target_generation: 7,
        }));
    }
}

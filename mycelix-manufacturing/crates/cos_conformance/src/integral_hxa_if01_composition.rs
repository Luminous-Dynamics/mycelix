//! Cross-layer HXA/IF01 composition witnesses.
//!
//! This module does not attempt to prove the entire production system.
//! It verifies that the human-experience boundary and the OAD -> COS
//! admission boundary cannot be composed into a stronger claim merely by
//! chaining individually valid-looking artifacts.
//!
//! Core composition rule:
//!   recommendation -> interpretation -> authorization -> execution
//! remains an explicit chain. No intermediate artifact silently supplies a
//! missing authority, provenance, uncertainty, or human-contestability link.

use super::integral_hxa_formal::{
    ActorKind, ActionState, HxaArtifact, HxaBoundary, HxaTransformation, ProvenanceClass,
    appeal_does_not_require_recommending_ai, consequential_action_requires_explicit_authorization,
    human_override_remains_possible, no_legitimacy_laundering,
    transformation_preserves_provenance, uncertainty_is_preserved,
};
use super::integral_oad_cos_formal::OBLIGATIONS as IF01_OBLIGATIONS;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CompositionDecision {
    AdmissibleReferencePath,
    RejectedMissingHumanAuthorization,
    RejectedMissingContestability,
    RejectedProvenanceBreak,
    RejectedUncertaintyLoss,
    RejectedAuthorityLaundering,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CrossLayerCase {
    pub hxa: HxaBoundary,
    pub transformation: HxaTransformation,
    pub if01_authorization_reference: bool,
    pub if01_semantic_admission: bool,
    pub execution_requested: bool,
}

pub fn evaluate_cross_layer(case: &CrossLayerCase) -> CompositionDecision {
    if !no_legitimacy_laundering(&case.hxa) {
        return CompositionDecision::RejectedAuthorityLaundering;
    }

    if case.execution_requested && !case.if01_authorization_reference {
        return CompositionDecision::RejectedMissingHumanAuthorization;
    }

    if case.execution_requested
        && !consequential_action_requires_explicit_authorization(&case.hxa)
    {
        return CompositionDecision::RejectedMissingHumanAuthorization;
    }

    if !appeal_does_not_require_recommending_ai(&case.hxa) {
        return CompositionDecision::RejectedMissingContestability;
    }

    if !transformation_preserves_provenance(&case.hxa.recommendation, &case.transformation) {
        return CompositionDecision::RejectedProvenanceBreak;
    }

    if !uncertainty_is_preserved(&case.hxa.recommendation, &case.transformation) {
        return CompositionDecision::RejectedUncertaintyLoss;
    }

    if case.execution_requested && !case.if01_semantic_admission {
        return CompositionDecision::RejectedMissingHumanAuthorization;
    }

    CompositionDecision::AdmissibleReferencePath
}

pub fn if01_and_hxa_are_both_bounded_reference_models() -> bool {
    !IF01_OBLIGATIONS.is_empty()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn recommendation() -> HxaArtifact {
        HxaArtifact {
            actor: ActorKind::Symthaea,
            class: ProvenanceClass::Recommendation,
            evidence_refs: 2,
            uncertainty_present: true,
            authorization_ref: false,
            can_be_reversed: true,
            independent_appeal: true,
            identity_disclosed: true,
        }
    }

    fn base() -> CrossLayerCase {
        CrossLayerCase {
            hxa: HxaBoundary {
                recommendation: recommendation(),
                action: ActionState::Proposed,
                explanation_is_authoritative: false,
                ai_can_authorize_itself: false,
                appeal_requires_ai: false,
                symthaea_available: true,
            },
            transformation: HxaTransformation {
                preserves_provenance: true,
                preserves_uncertainty: true,
                preserves_disagreement: true,
            },
            if01_authorization_reference: false,
            if01_semantic_admission: true,
            execution_requested: false,
        }
    }

    #[test]
    fn valid_reference_path_remains_bounded() {
        let c = base();
        assert_eq!(evaluate_cross_layer(&c), CompositionDecision::AdmissibleReferencePath);
        assert!(if01_and_hxa_are_both_bounded_reference_models());
    }

    #[test]
    fn execution_cannot_skip_human_authorization() {
        let mut c = base();
        c.execution_requested = true;
        c.hxa.action = ActionState::Executed;
        assert_eq!(
            evaluate_cross_layer(&c),
            CompositionDecision::RejectedMissingHumanAuthorization
        );
    }

    #[test]
    fn if01_authorization_reference_is_not_enough_to_launder_ai_authority() {
        let mut c = base();
        c.execution_requested = true;
        c.hxa.action = ActionState::Authorized;
        c.hxa.recommendation.authorization_ref = true;
        c.if01_authorization_reference = true;
        c.hxa.ai_can_authorize_itself = true;
        assert_eq!(
            evaluate_cross_layer(&c),
            CompositionDecision::RejectedAuthorityLaundering
        );
    }

    #[test]
    fn provenance_break_blocks_composition() {
        let mut c = base();
        c.transformation.preserves_provenance = false;
        assert_eq!(
            evaluate_cross_layer(&c),
            CompositionDecision::RejectedProvenanceBreak
        );
    }

    #[test]
    fn uncertainty_loss_blocks_composition() {
        let mut c = base();
        c.transformation.preserves_uncertainty = false;
        assert_eq!(
            evaluate_cross_layer(&c),
            CompositionDecision::RejectedUncertaintyLoss
        );
    }

    #[test]
    fn appeal_cannot_depend_on_the_recommending_ai() {
        let mut c = base();
        c.hxa.appeal_requires_ai = true;
        assert_eq!(
            evaluate_cross_layer(&c),
            CompositionDecision::RejectedMissingContestability
        );
    }

    #[test]
    fn human_override_is_available_before_execution() {
        let c = base();
        assert!(human_override_remains_possible(&c.hxa));
    }

    #[test]
    fn missing_semantic_admission_blocks_execution() {
        let mut c = base();
        c.execution_requested = true;
        c.hxa.action = ActionState::Authorized;
        c.hxa.recommendation.authorization_ref = true;
        c.if01_authorization_reference = true;
        c.if01_semantic_admission = false;
        assert_eq!(
            evaluate_cross_layer(&c),
            CompositionDecision::RejectedMissingHumanAuthorization
        );
    }

    #[test]
    fn composition_does_not_claim_human_outcomes() {
        // Passing this suite says only that encoded cross-layer boundary
        // conditions hold. It says nothing about comprehension, happiness,
        // satisfaction, trust, or flourishing.
        assert!(if01_and_hxa_are_both_bounded_reference_models());
    }
}

//! HXA formal-reference witnesses for the human-experience boundary.
//!
//! These witnesses formalize only properties that can be enforced by the
//! machine-readable boundary. They do NOT prove that people understand,
//! flourish, feel safe, or are satisfied. Those claims require participant
//! evidence and longitudinal evaluation.
//!
//! The central rule is:
//!   Symthaea may assist cognition; Mycelix may preserve authority/provenance;
//!   neither may silently convert recommendation into legitimate human choice.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProvenanceClass {
    Observation,
    Interpretation,
    Recommendation,
    Decision,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActorKind {
    Human,
    Symthaea,
    OtherSystem,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ActionState {
    Proposed,
    Authorized,
    Rejected,
    Executed,
    Reversible,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HxaArtifact {
    pub actor: ActorKind,
    pub class: ProvenanceClass,
    pub evidence_refs: u8,
    pub uncertainty_present: bool,
    pub authorization_ref: bool,
    pub can_be_reversed: bool,
    pub independent_appeal: bool,
    pub identity_disclosed: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HxaTransformation {
    pub preserves_provenance: bool,
    pub preserves_uncertainty: bool,
    pub preserves_disagreement: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HxaBoundary {
    pub recommendation: HxaArtifact,
    pub action: ActionState,
    pub explanation_is_authoritative: bool,
    pub ai_can_authorize_itself: bool,
    pub appeal_requires_ai: bool,
    pub symthaea_available: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct HxaObligation {
    pub id: &'static str,
    pub proposition: &'static str,
    pub witness: &'static str,
    pub claim_ceiling: &'static str,
}

pub const OBLIGATIONS: [HxaObligation; 12] = [
    HxaObligation { id: "HXA-I01", proposition: "A recommendation is not authority.", witness: "recommendation_never_becomes_authority", claim_ceiling: "Recommendation/authority separation only." },
    HxaObligation { id: "HXA-I02", proposition: "An explanation is not evidence merely because it is generated.", witness: "explanation_does_not_create_evidence", claim_ceiling: "Explanation/evidence distinction only." },
    HxaObligation { id: "HXA-I03", proposition: "An explanation cannot mutate authoritative records.", witness: "explanation_cannot_mutate_authoritative_state", claim_ceiling: "Non-authoritative explanation semantics only." },
    HxaObligation { id: "HXA-I04", proposition: "Uncertainty must survive presentation/transformation.", witness: "uncertainty_is_preserved", claim_ceiling: "Uncertainty-preservation semantics only." },
    HxaObligation { id: "HXA-I05", proposition: "Consequential action requires explicit authorization.", witness: "consequential_action_requires_explicit_authorization", claim_ceiling: "Authorization-boundary semantics only." },
    HxaObligation { id: "HXA-I06", proposition: "A human can override an AI recommendation.", witness: "human_override_remains_possible", claim_ceiling: "Override-path availability only." },
    HxaObligation { id: "HXA-I07", proposition: "Contestability does not require the same AI that made the recommendation.", witness: "appeal_does_not_require_recommending_ai", claim_ceiling: "AI-independent appeal-path semantics only." },
    HxaObligation { id: "HXA-I08", proposition: "Accessibility/translation transformations preserve provenance.", witness: "transformation_preserves_provenance", claim_ceiling: "Transformation provenance semantics only." },
    HxaObligation { id: "HXA-I09", proposition: "Disagreement is not silently erased.", witness: "disagreement_is_preserved", claim_ceiling: "Disagreement preservation semantics only." },
    HxaObligation { id: "HXA-I10", proposition: "AI identity is disclosed at the human boundary.", witness: "ai_identity_is_disclosed", claim_ceiling: "Actor identity disclosure semantics only." },
    HxaObligation { id: "HXA-I11", proposition: "The human boundary degrades safely when Symthaea is unavailable.", witness: "safe_degradation_without_symthaea", claim_ceiling: "Assistive-system independence semantics only." },
    HxaObligation { id: "HXA-I12", proposition: "No generated artifact can launder legitimacy into a decision.", witness: "no_legitimacy_laundering", claim_ceiling: "Legitimacy-boundary semantics only." },
];

pub fn recommendation_never_becomes_authority(a: &HxaArtifact) -> bool {
    // An authorization reference may authorize a consequential action, but it
    // never changes the provenance class of the recommendation itself.
    a.class == ProvenanceClass::Recommendation
}

pub fn explanation_does_not_create_evidence(explanation: &HxaArtifact) -> bool {
    explanation.class != ProvenanceClass::Observation
}

pub fn explanation_cannot_mutate_authoritative_state(boundary: &HxaBoundary) -> bool {
    !boundary.explanation_is_authoritative
}

pub fn uncertainty_is_preserved(before: &HxaArtifact, after: &HxaTransformation) -> bool {
    !before.uncertainty_present || after.preserves_uncertainty
}

pub fn consequential_action_requires_explicit_authorization(boundary: &HxaBoundary) -> bool {
    if matches!(boundary.action, ActionState::Authorized | ActionState::Executed) {
        boundary.recommendation.authorization_ref
    } else {
        true
    }
}

pub fn human_override_remains_possible(boundary: &HxaBoundary) -> bool {
    boundary.recommendation.actor == ActorKind::Symthaea
        && matches!(boundary.action, ActionState::Proposed | ActionState::Rejected | ActionState::Reversible)
}

pub fn appeal_does_not_require_recommending_ai(boundary: &HxaBoundary) -> bool {
    !boundary.appeal_requires_ai && boundary.recommendation.independent_appeal
}

pub fn transformation_preserves_provenance(
    source: &HxaArtifact,
    transformation: &HxaTransformation,
) -> bool {
    source.evidence_refs == 0 || transformation.preserves_provenance
}

pub fn disagreement_is_preserved(transformation: &HxaTransformation) -> bool {
    transformation.preserves_disagreement
}

pub fn ai_identity_is_disclosed(a: &HxaArtifact) -> bool {
    a.actor != ActorKind::Symthaea || a.identity_disclosed
}

pub fn safe_degradation_without_symthaea(boundary: &HxaBoundary) -> bool {
    !boundary.symthaea_available
        && boundary.action != ActionState::Executed
        && !boundary.ai_can_authorize_itself
}

pub fn no_legitimacy_laundering(boundary: &HxaBoundary) -> bool {
    !boundary.ai_can_authorize_itself
        && !boundary.explanation_is_authoritative
        && boundary.recommendation.class == ProvenanceClass::Recommendation
}

pub fn all_machine_checkable_obligations_have_witnesses() -> bool {
    OBLIGATIONS.iter().all(|o| !o.id.is_empty() && !o.proposition.is_empty() && !o.witness.is_empty())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ai_recommendation() -> HxaArtifact {
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

    fn safe_boundary() -> HxaBoundary {
        HxaBoundary {
            recommendation: ai_recommendation(),
            action: ActionState::Proposed,
            explanation_is_authoritative: false,
            ai_can_authorize_itself: false,
            appeal_requires_ai: false,
            symthaea_available: true,
        }
    }

    #[test]
    fn all_machine_checkable_obligations_have_witnesses() {
        assert_eq!(OBLIGATIONS.len(), 12);
        assert!(all_machine_checkable_obligations_have_witnesses());
    }

    #[test]
    fn recommendation_never_mints_authority() {
        let mut a = ai_recommendation();
        assert!(recommendation_never_becomes_authority(&a));
        a.authorization_ref = true;
        assert!(recommendation_never_becomes_authority(&a));
    }

    #[test]
    fn explanation_cannot_become_evidence_by_generation() {
        let a = HxaArtifact { class: ProvenanceClass::Interpretation, ..ai_recommendation() };
        assert!(explanation_does_not_create_evidence(&a));
    }

    #[test]
    fn explanation_cannot_mutate_authoritative_state() {
        let mut b = safe_boundary();
        assert!(explanation_cannot_mutate_authoritative_state(&b));
        b.explanation_is_authoritative = true;
        assert!(!explanation_cannot_mutate_authoritative_state(&b));
    }

    #[test]
    fn uncertainty_must_survive_transformation() {
        let a = ai_recommendation();
        assert!(uncertainty_is_preserved(&a, &HxaTransformation {
            preserves_provenance: true,
            preserves_uncertainty: true,
            preserves_disagreement: true,
        }));
        assert!(!uncertainty_is_preserved(&a, &HxaTransformation {
            preserves_provenance: true,
            preserves_uncertainty: false,
            preserves_disagreement: true,
        }));
    }

    #[test]
    fn consequential_action_requires_explicit_authorization() {
        let mut b = safe_boundary();
        b.action = ActionState::Executed;
        assert!(!consequential_action_requires_explicit_authorization(&b));
        b.recommendation.authorization_ref = true;
        assert!(consequential_action_requires_explicit_authorization(&b));
    }

    #[test]
    fn human_override_remains_possible() {
        assert!(human_override_remains_possible(&safe_boundary()));
        let mut b = safe_boundary();
        b.action = ActionState::Authorized;
        assert!(!human_override_remains_possible(&b));
    }

    #[test]
    fn appeal_is_not_locked_to_recommending_ai() {
        assert!(appeal_does_not_require_recommending_ai(&safe_boundary()));
        let mut b = safe_boundary();
        b.appeal_requires_ai = true;
        assert!(!appeal_does_not_require_recommending_ai(&b));
    }

    #[test]
    fn translation_preserves_provenance() {
        let a = ai_recommendation();
        assert!(transformation_preserves_provenance(&a, &HxaTransformation {
            preserves_provenance: true,
            preserves_uncertainty: true,
            preserves_disagreement: true,
        }));
        assert!(!transformation_preserves_provenance(&a, &HxaTransformation {
            preserves_provenance: false,
            preserves_uncertainty: true,
            preserves_disagreement: true,
        }));
    }

    #[test]
    fn disagreement_cannot_be_silently_erased() {
        assert!(disagreement_is_preserved(&HxaTransformation {
            preserves_provenance: true,
            preserves_uncertainty: true,
            preserves_disagreement: true,
        }));
        assert!(!disagreement_is_preserved(&HxaTransformation {
            preserves_provenance: true,
            preserves_uncertainty: true,
            preserves_disagreement: false,
        }));
    }

    #[test]
    fn ai_identity_must_be_disclosed() {
        assert!(ai_identity_is_disclosed(&ai_recommendation()));
        let mut a = ai_recommendation();
        a.identity_disclosed = false;
        assert!(!ai_identity_is_disclosed(&a));
    }

    #[test]
    fn safe_degradation_without_symthaea() {
        let mut b = safe_boundary();
        b.symthaea_available = false;
        assert!(safe_degradation_without_symthaea(&b));
        b.action = ActionState::Executed;
        assert!(!safe_degradation_without_symthaea(&b));
    }

    #[test]
    fn no_legitimacy_laundering() {
        assert!(no_legitimacy_laundering(&safe_boundary()));
        let mut b = safe_boundary();
        b.ai_can_authorize_itself = true;
        assert!(!no_legitimacy_laundering(&b));
    }

    #[test]
    fn human_evidence_is_not_claimed_by_machine_witnesses() {
        // These tests establish boundary semantics only. They intentionally do
        // not assert comprehension, satisfaction, flourishing, or happiness.
        assert!(all_machine_checkable_obligations_have_witnesses());
    }
}

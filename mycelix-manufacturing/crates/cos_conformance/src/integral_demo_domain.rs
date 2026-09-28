//! Minimal domain seam for the Integral reference-node demo.
//!
//! This is deliberately a composition model, not a second implementation of
//! Integral's five systems. It gives the demo explicit boundaries between:
//! proposal/design, decision, authorization, execution intent, observation,
//! assessment/recommendation, outcome, and appeal.
//!
//! Evidence ceiling: bounded reference model. These types do not establish
//! production correctness, Integral ratification, economic validity, or human
//! flourishing.

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ProvenanceClass {
    Proposal,
    Design,
    Decision,
    Authorization,
    ExecutionIntent,
    Observation,
    Assessment,
    Recommendation,
    Outcome,
    Appeal,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SourceKind {
    Local,
    Foreign,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DemoDecision {
    Draft,
    Accepted,
    Rejected,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Artifact {
    pub class: ProvenanceClass,
    pub source: SourceKind,
    pub generation: u64,
    pub uncertainty_present: bool,
    pub explicit_human_authorization: bool,
    pub reversible: bool,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DemoTransition {
    pub from: ProvenanceClass,
    pub to: ProvenanceClass,
    pub authorized: bool,
    pub semantic_admission: bool,
    pub source_generation: u64,
    pub target_generation: u64,
}

pub fn transition_allowed(t: &DemoTransition) -> bool {
    if t.target_generation < t.source_generation {
        return false;
    }

    match (t.from, t.to) {
        (ProvenanceClass::Proposal, ProvenanceClass::Design) => true,
        (ProvenanceClass::Design, ProvenanceClass::Decision) => true,
        (ProvenanceClass::Decision, ProvenanceClass::Authorization) => t.authorized,
        (ProvenanceClass::Authorization, ProvenanceClass::ExecutionIntent) => {
            t.authorized && t.semantic_admission
        }
        (ProvenanceClass::ExecutionIntent, ProvenanceClass::Observation) => true,
        (ProvenanceClass::Observation, ProvenanceClass::Assessment) => true,
        (ProvenanceClass::Assessment, ProvenanceClass::Recommendation) => true,
        (ProvenanceClass::Observation, ProvenanceClass::Outcome) => true,
        (_, ProvenanceClass::Appeal) => true,
        // In particular, a recommendation cannot become a decision or
        // authorization merely by relabeling or attaching metadata.
        _ => false,
    }
}

pub fn recommendation_can_inform_decision(artifact: &Artifact) -> bool {
    artifact.class == ProvenanceClass::Recommendation
        && artifact.uncertainty_present
        && !artifact.explicit_human_authorization
}

pub fn authorization_is_not_execution(artifact: &Artifact) -> bool {
    artifact.class == ProvenanceClass::Authorization
        && artifact.explicit_human_authorization
        && artifact.reversible
}

pub fn observation_is_not_qualification(artifact: &Artifact) -> bool {
    artifact.class == ProvenanceClass::Observation
}

pub fn foreign_origin_is_preserved(artifact: &Artifact) -> bool {
    artifact.source == SourceKind::Foreign
}

pub fn appeal_is_independent_of_recommender(artifact: &Artifact) -> bool {
    artifact.class == ProvenanceClass::Appeal && artifact.reversible
}

#[cfg(test)]
mod tests {
    use super::*;

    fn artifact(class: ProvenanceClass) -> Artifact {
        Artifact {
            class,
            source: SourceKind::Local,
            generation: 1,
            uncertainty_present: true,
            explicit_human_authorization: class == ProvenanceClass::Authorization,
            reversible: true,
        }
    }

    #[test]
    fn normal_reference_node_chain_is_explicit() {
        let stages = [
            (ProvenanceClass::Proposal, ProvenanceClass::Design, false),
            (ProvenanceClass::Design, ProvenanceClass::Decision, false),
            (ProvenanceClass::Decision, ProvenanceClass::Authorization, true),
            (ProvenanceClass::Authorization, ProvenanceClass::ExecutionIntent, true),
            (ProvenanceClass::ExecutionIntent, ProvenanceClass::Observation, false),
            (ProvenanceClass::Observation, ProvenanceClass::Assessment, false),
            (ProvenanceClass::Assessment, ProvenanceClass::Recommendation, false),
        ];

        for (from, to, authorized in stages {
            assert!(transition_allowed(&DemoTransition {
                from,
                to,
                authorized,
                semantic_admission: true,
                source_generation: 1,
                target_generation: 1,
            }));
        }
    }

    #[test]
    fn recommendation_cannot_become_authority() {
        let r = artifact(ProvenanceClass::Recommendation);
        assert!(recommendation_can_inform_decision(&r));
        assert!(!transition_allowed(&DemoTransition {
            from: ProvenanceClass::Recommendation,
            to: ProvenanceClass::Authorization,
            authorized: true,
            semantic_admission: true,
            source_generation: 1,
            target_generation: 1,
        }));
    }

    #[test]
    fn authorization_does_not_equal_execution() {
        let a = artifact(ProvenanceClass::Authorization);
        assert!(authorization_is_not_execution(&a));
        assert!(!transition_allowed(&DemoTransition {
            from: ProvenanceClass::Decision,
            to: ProvenanceClass::ExecutionIntent,
            authorized: true,
            semantic_admission: true,
            source_generation: 1,
            target_generation: 1,
        }));
    }

    #[test]
    fn execution_requires_semantic_admission() {
        assert!(!transition_allowed(&DemoTransition {
            from: ProvenanceClass::Authorization,
            to: ProvenanceClass::ExecutionIntent,
            authorized: true,
            semantic_admission: false,
            source_generation: 1,
            target_generation: 1,
        }));
    }

    #[test]
    fn stale_generation_cannot_move_forward() {
        assert!(!transition_allowed(&DemoTransition {
            from: ProvenanceClass::Authorization,
            to: ProvenanceClass::ExecutionIntent,
            authorized: true,
            semantic_admission: true,
            source_generation: 2,
            target_generation: 1,
        }));
    }

    #[test]
    fn observation_is_not_qualification() {
        assert!(observation_is_not_qualification(&artifact(ProvenanceClass::Observation)));
    }

    #[test]
    fn foreign_origin_is_not_rewritten() {
        let foreign = Artifact {
            source: SourceKind::Foreign,
            ..artifact(ProvenanceClass::Observation)
        };
        assert!(foreign_origin_is_preserved(&foreign));
    }

    #[test]
    fn appeal_is_a_distinct_recovery_path() {
        assert!(appeal_is_independent_of_recommender(&artifact(ProvenanceClass::Appeal)));
    }
}

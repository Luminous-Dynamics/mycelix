// Copyright (C) 2024-2026 Tristan Stoltz / Luminous Dynamics
// SPDX-License-Identifier: AGPL-3.0-or-later
//! Fail-closed exact-representation knowledge-use policy theorem.
//!
//! STEW-003 classifies use actions for one exact STEW-001 representation. It
//! does not authenticate a policy issuer, establish stewardship legitimacy,
//! evaluate referenced constraints/duties, or produce final runtime authority.

#![forbid(unsafe_code)]

use core::fmt;
use mycelix_stewardship_core::{
    CanonicalIdErrorV1, CanonicalIdV1, StewardedSubjectIdentityV1,
};

/// Stable profile identifier for STEW-003.
pub const KNOWLEDGE_USE_POLICY_PROFILE_V1: &str = "mycelix/knowledge-use-policy/v1";

/// Maximum rules admitted by one v1 policy.
pub const MAX_POLICY_RULES_V1: usize = 64;
/// Maximum opaque constraint references on one permission rule.
pub const MAX_CONSTRAINT_REFS_PER_RULE_V1: usize = 32;
/// Maximum opaque duty references on one permission rule.
pub const MAX_DUTY_REFS_PER_RULE_V1: usize = 32;

/// Closed v1 action vocabulary.
///
/// These actions remain independent capabilities; no variant implies another.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub enum KnowledgeUseActionV1 {
    Discover,
    InspectMetadata,
    Retrieve,
    View,
    Quote,
    Translate,
    Perform,
    Reproduce,
    Transform,
    Research,
    Commercialize,
    TrainAi,
    InferFrom,
    GenerateDerivative,
    Redistribute,
    Archive,
    Disclose,
}

/// Opaque reference to a constraint evaluated by a later authority layer.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct ConstraintRefV1(CanonicalIdV1);

impl ConstraintRefV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        CanonicalIdV1::new(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

/// Opaque reference to a duty/obligation evaluated by a later authority layer.
#[derive(Debug, Clone, PartialEq, Eq, PartialOrd, Ord, Hash)]
pub struct DutyRefV1(CanonicalIdV1);

impl DutyRefV1 {
    pub fn new(value: impl Into<String>) -> Result<Self, CanonicalIdErrorV1> {
        CanonicalIdV1::new(value).map(Self)
    }

    pub fn as_str(&self) -> &str {
        self.0.as_str()
    }
}

/// Rule-construction failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PolicyRuleErrorV1 {
    TooManyConstraintReferences,
    TooManyDutyReferences,
    DuplicateConstraintReference,
    DuplicateDutyReference,
}

impl fmt::Display for PolicyRuleErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooManyConstraintReferences => {
                f.write_str("too many constraint references for v1 permission rule")
            }
            Self::TooManyDutyReferences => {
                f.write_str("too many duty references for v1 permission rule")
            }
            Self::DuplicateConstraintReference => {
                f.write_str("duplicate constraint reference in permission rule")
            }
            Self::DuplicateDutyReference => {
                f.write_str("duplicate duty reference in permission rule")
            }
        }
    }
}

/// One closed-world v1 policy rule.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum KnowledgeUseRuleV1 {
    /// A permission candidate. Referenced constraints/duties are not evaluated
    /// by this theorem and therefore this is not final authorization.
    Permit {
        action: KnowledgeUseActionV1,
        constraints: Vec<ConstraintRefV1>,
        duties: Vec<DutyRefV1>,
    },
    /// Absolute v1 prohibition. Conditional prohibitions are deferred to a
    /// richer later profile/ODRL mapping so they cannot be mis-evaluated here.
    Prohibit { action: KnowledgeUseActionV1 },
}

impl KnowledgeUseRuleV1 {
    /// Construct a bounded permission candidate.
    pub fn permit(
        action: KnowledgeUseActionV1,
        constraints: Vec<ConstraintRefV1>,
        duties: Vec<DutyRefV1>,
    ) -> Result<Self, PolicyRuleErrorV1> {
        if constraints.len() > MAX_CONSTRAINT_REFS_PER_RULE_V1 {
            return Err(PolicyRuleErrorV1::TooManyConstraintReferences);
        }
        if duties.len() > MAX_DUTY_REFS_PER_RULE_V1 {
            return Err(PolicyRuleErrorV1::TooManyDutyReferences);
        }
        for (index, reference) in constraints.iter().enumerate() {
            if constraints[..index].contains(reference) {
                return Err(PolicyRuleErrorV1::DuplicateConstraintReference);
            }
        }
        for (index, reference) in duties.iter().enumerate() {
            if duties[..index].contains(reference) {
                return Err(PolicyRuleErrorV1::DuplicateDutyReference);
            }
        }
        Ok(Self::Permit {
            action,
            constraints,
            duties,
        })
    }

    /// Construct an absolute prohibition.
    pub const fn prohibit(action: KnowledgeUseActionV1) -> Self {
        Self::Prohibit { action }
    }

    /// Action governed by this rule.
    pub const fn action(&self) -> KnowledgeUseActionV1 {
        match self {
            Self::Permit { action, .. } | Self::Prohibit { action } => *action,
        }
    }
}

/// Policy-construction failure.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum KnowledgeUsePolicyErrorV1 {
    TooManyRules,
    DuplicatePermissionForAction,
    DuplicateProhibitionForAction,
}

impl fmt::Display for KnowledgeUsePolicyErrorV1 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::TooManyRules => f.write_str("too many policy rules for v1"),
            Self::DuplicatePermissionForAction => {
                f.write_str("duplicate permission rule for one v1 action")
            }
            Self::DuplicateProhibitionForAction => {
                f.write_str("duplicate prohibition rule for one v1 action")
            }
        }
    }
}

/// Fail-closed classification of one requested action.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum ActionDispositionV1<'a> {
    /// An absolute prohibition exists. Prohibition dominates any permission.
    Prohibited,
    /// A permission rule exists, but referenced constraints/duties still need
    /// independent evaluation before runtime authorization can be granted.
    PermissionCandidate {
        constraints: &'a [ConstraintRefV1],
        duties: &'a [DutyRefV1],
    },
    /// No rule exists for the action. v1 fails closed.
    DeniedUnspecified,
}

/// Exact-representation use policy.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct KnowledgeUsePolicyV1 {
    policy_id: CanonicalIdV1,
    target: StewardedSubjectIdentityV1,
    rules: Vec<KnowledgeUseRuleV1>,
}

impl KnowledgeUsePolicyV1 {
    /// Construct a fail-closed policy bound to one exact representation.
    pub fn new(
        policy_id: CanonicalIdV1,
        target: StewardedSubjectIdentityV1,
        rules: Vec<KnowledgeUseRuleV1>,
    ) -> Result<Self, KnowledgeUsePolicyErrorV1> {
        if rules.len() > MAX_POLICY_RULES_V1 {
            return Err(KnowledgeUsePolicyErrorV1::TooManyRules);
        }

        for (index, rule) in rules.iter().enumerate() {
            match rule {
                KnowledgeUseRuleV1::Permit { action, .. } => {
                    if rules[..index].iter().any(|previous| {
                        matches!(
                            previous,
                            KnowledgeUseRuleV1::Permit {
                                action: previous_action,
                                ..
                            } if previous_action == action
                        )
                    }) {
                        return Err(KnowledgeUsePolicyErrorV1::DuplicatePermissionForAction);
                    }
                }
                KnowledgeUseRuleV1::Prohibit { action } => {
                    if rules[..index].iter().any(|previous| {
                        matches!(
                            previous,
                            KnowledgeUseRuleV1::Prohibit {
                                action: previous_action
                            } if previous_action == action
                        )
                    }) {
                        return Err(KnowledgeUsePolicyErrorV1::DuplicateProhibitionForAction);
                    }
                }
            }
        }

        Ok(Self {
            policy_id,
            target,
            rules,
        })
    }

    /// Opaque identity of this policy object/version.
    pub fn policy_id(&self) -> &CanonicalIdV1 {
        &self.policy_id
    }

    /// Exact STEW-001 representation governed by this policy.
    pub fn target(&self) -> &StewardedSubjectIdentityV1 {
        &self.target
    }

    /// Stored rules. Their order does not alter prohibition dominance.
    pub fn rules(&self) -> &[KnowledgeUseRuleV1] {
        &self.rules
    }

    /// Classify one action without evaluating external constraints or duties.
    pub fn classify_action(&self, requested: KnowledgeUseActionV1) -> ActionDispositionV1<'_> {
        if self.rules.iter().any(|rule| {
            matches!(
                rule,
                KnowledgeUseRuleV1::Prohibit { action } if *action == requested
            )
        }) {
            return ActionDispositionV1::Prohibited;
        }

        if let Some((constraints, duties)) = self.rules.iter().find_map(|rule| match rule {
            KnowledgeUseRuleV1::Permit {
                action,
                constraints,
                duties,
            } if *action == requested => Some((constraints.as_slice(), duties.as_slice())),
            _ => None,
        }) {
            return ActionDispositionV1::PermissionCandidate {
                constraints,
                duties,
            };
        }

        ActionDispositionV1::DeniedUnspecified
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use mycelix_stewardship_core::{
        ContentDigestV1, DigestAlgorithmV1, RepresentationIdV1, RepresentationKindV1,
        RevisionIdV1, StewardedSubjectIdV1,
    };

    fn target() -> StewardedSubjectIdentityV1 {
        StewardedSubjectIdentityV1 {
            subject: StewardedSubjectIdV1::new("subject:work:example").unwrap(),
            revision: RevisionIdV1::new("revision:1").unwrap(),
            representation: RepresentationIdV1::new("representation:recording:1").unwrap(),
            kind: RepresentationKindV1::Audio,
            content_digest: ContentDigestV1::new(DigestAlgorithmV1::Blake3_256, [5; 32]),
        }
    }

    fn policy(rules: Vec<KnowledgeUseRuleV1>) -> KnowledgeUsePolicyV1 {
        KnowledgeUsePolicyV1::new(
            CanonicalIdV1::new("policy:use:example:v1").unwrap(),
            target(),
            rules,
        )
        .unwrap()
    }

    fn permit(action: KnowledgeUseActionV1) -> KnowledgeUseRuleV1 {
        KnowledgeUseRuleV1::permit(action, vec![], vec![]).unwrap()
    }

    #[test]
    fn view_permission_does_not_imply_ai_training_permission() {
        let policy = policy(vec![permit(KnowledgeUseActionV1::View)]);
        assert!(matches!(
            policy.classify_action(KnowledgeUseActionV1::View),
            ActionDispositionV1::PermissionCandidate { .. }
        ));
        assert_eq!(
            policy.classify_action(KnowledgeUseActionV1::TrainAi),
            ActionDispositionV1::DeniedUnspecified
        );
    }

    #[test]
    fn research_permission_does_not_imply_commercialization() {
        let policy = policy(vec![permit(KnowledgeUseActionV1::Research)]);
        assert_eq!(
            policy.classify_action(KnowledgeUseActionV1::Commercialize),
            ActionDispositionV1::DeniedUnspecified
        );
    }

    #[test]
    fn retrieve_permission_does_not_imply_disclosure_or_redistribution() {
        let policy = policy(vec![permit(KnowledgeUseActionV1::Retrieve)]);
        assert_eq!(
            policy.classify_action(KnowledgeUseActionV1::Disclose),
            ActionDispositionV1::DeniedUnspecified
        );
        assert_eq!(
            policy.classify_action(KnowledgeUseActionV1::Redistribute),
            ActionDispositionV1::DeniedUnspecified
        );
    }

    #[test]
    fn derivative_permission_does_not_imply_redistribution() {
        let policy = policy(vec![permit(KnowledgeUseActionV1::GenerateDerivative)]);
        assert_eq!(
            policy.classify_action(KnowledgeUseActionV1::Redistribute),
            ActionDispositionV1::DeniedUnspecified
        );
    }

    #[test]
    fn unspecified_action_always_fails_closed() {
        let policy = policy(vec![]);
        assert_eq!(
            policy.classify_action(KnowledgeUseActionV1::Discover),
            ActionDispositionV1::DeniedUnspecified
        );
    }

    #[test]
    fn explicit_prohibition_dominates_permission() {
        let policy = policy(vec![
            permit(KnowledgeUseActionV1::TrainAi),
            KnowledgeUseRuleV1::prohibit(KnowledgeUseActionV1::TrainAi),
        ]);
        assert_eq!(
            policy.classify_action(KnowledgeUseActionV1::TrainAi),
            ActionDispositionV1::Prohibited
        );
    }

    #[test]
    fn constraints_and_duties_remain_unverified_references() {
        let rule = KnowledgeUseRuleV1::permit(
            KnowledgeUseActionV1::Research,
            vec![ConstraintRefV1::new("constraint:community-member").unwrap()],
            vec![DutyRefV1::new("duty:return-results").unwrap()],
        )
        .unwrap();
        let policy = policy(vec![rule]);
        match policy.classify_action(KnowledgeUseActionV1::Research) {
            ActionDispositionV1::PermissionCandidate {
                constraints,
                duties,
            } => {
                assert_eq!(constraints[0].as_str(), "constraint:community-member");
                assert_eq!(duties[0].as_str(), "duty:return-results");
            }
            other => panic!("unexpected disposition: {other:?}"),
        }
    }

    #[test]
    fn duplicate_permission_and_prohibition_rules_are_rejected_independently() {
        let id = CanonicalIdV1::new("policy:duplicate-test").unwrap();
        assert_eq!(
            KnowledgeUsePolicyV1::new(
                id.clone(),
                target(),
                vec![
                    permit(KnowledgeUseActionV1::View),
                    permit(KnowledgeUseActionV1::View),
                ],
            ),
            Err(KnowledgeUsePolicyErrorV1::DuplicatePermissionForAction)
        );
        assert_eq!(
            KnowledgeUsePolicyV1::new(
                id,
                target(),
                vec![
                    KnowledgeUseRuleV1::prohibit(KnowledgeUseActionV1::View),
                    KnowledgeUseRuleV1::prohibit(KnowledgeUseActionV1::View),
                ],
            ),
            Err(KnowledgeUsePolicyErrorV1::DuplicateProhibitionForAction)
        );
    }

    #[test]
    fn duplicate_constraint_and_duty_references_are_rejected() {
        let constraint = ConstraintRefV1::new("constraint:x").unwrap();
        assert_eq!(
            KnowledgeUseRuleV1::permit(
                KnowledgeUseActionV1::View,
                vec![constraint.clone(), constraint],
                vec![],
            ),
            Err(PolicyRuleErrorV1::DuplicateConstraintReference)
        );

        let duty = DutyRefV1::new("duty:x").unwrap();
        assert_eq!(
            KnowledgeUseRuleV1::permit(
                KnowledgeUseActionV1::View,
                vec![],
                vec![duty.clone(), duty],
            ),
            Err(PolicyRuleErrorV1::DuplicateDutyReference)
        );
    }

    #[test]
    fn policy_is_bound_to_exact_representation_identity() {
        let policy = policy(vec![permit(KnowledgeUseActionV1::Archive)]);
        assert_eq!(policy.target(), &target());
    }

    #[test]
    fn profile_identifier_is_frozen() {
        assert_eq!(
            KNOWLEDGE_USE_POLICY_PROFILE_V1,
            "mycelix/knowledge-use-policy/v1"
        );
    }
}

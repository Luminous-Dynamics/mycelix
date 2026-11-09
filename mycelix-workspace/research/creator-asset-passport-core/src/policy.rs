//! Narrow ODRL 2.2 policy-conflict semantics for CSCP.
//!
//! This module resolves only the conflict between already-matched Permission
//! and Prohibition rules. It does not match resources, actors, actions,
//! purposes, constraints or duties; those remain the responsibility of the
//! calling policy adapter and existing CSCP grant evaluator.
//!
//! ODRL 2.2 defines perm, prohibit, and invalid. When no strategy is
//! provided, invalid is the default. If conflicting rules are composed with
//! multiple conflict-strategy values, the policy is invalid. The CSCP profile
//! uses invalid unless an explicitly versioned profile says otherwise.
//!
//! Reference: <https://www.w3.org/TR/odrl-model/#policy-conflict-strategy>

#[derive(Clone, Copy, Debug, Eq, PartialEq, Ord, PartialOrd, Hash)]
pub enum OdrConflictStrategy {
    /// A matching Permission overrides a conflicting Prohibition.
    Perm,
    /// A matching Prohibition overrides a conflicting Permission.
    Prohibit,
    /// Any conflict invalidates the entire composed policy.
    Invalid,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PolicyRuleEffect {
    Permission,
    Prohibition,
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum PolicyCompositionDecision {
    /// No matching Permission or Prohibition was supplied to the resolver.
    NoApplicableRule,
    /// The composed, matching rules resolve to an effective Permission.
    Permission,
    /// The composed, matching rules resolve to an effective Prohibition.
    Prohibition,
    /// Conflicting rules make the policy invalid under the selected strategy.
    InvalidPolicy,
}

/// Resolve conflicts among rules that have already been matched to the same
/// resource, actor, action, purpose, audience, channel and validity interval.
///
/// strategies is the set of ODRL conflict-strategy values present after
/// policy merge/inheritance. No value means ODRL's default invalid behavior.
///
/// This function is deliberately not a full policy evaluator. Callers must
/// establish rule applicability and satisfy all other constraints/duties
/// before treating Permission as an authorization result.
pub fn resolve_odrl_conflict(
    effects: &[PolicyRuleEffect],
    strategies: &[OdrConflictStrategy],
) -> PolicyCompositionDecision {
    let has_permission = effects.contains(&PolicyRuleEffect::Permission);
    let has_prohibition = effects.contains(&PolicyRuleEffect::Prohibition);

    match (has_permission, has_prohibition) {
        (false, false) => PolicyCompositionDecision::NoApplicableRule,
        (true, false) => PolicyCompositionDecision::Permission,
        (false, true) => PolicyCompositionDecision::Prohibition,
        (true, true) => {
            // ODRL requires an invalid policy when multiple conflict values
            // are present and the merged rules conflict. Missing strategy
            // selects the default invalid behavior.
            if strategies.len() > 1 {
                return PolicyCompositionDecision::InvalidPolicy;
            }

            match strategies.first().copied().unwrap_or(OdrConflictStrategy::Invalid) {
                OdrConflictStrategy::Perm => PolicyCompositionDecision::Permission,
                OdrConflictStrategy::Prohibit => PolicyCompositionDecision::Prohibition,
                OdrConflictStrategy::Invalid => PolicyCompositionDecision::InvalidPolicy,
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn no_matching_rule_returns_no_applicable_rule() {
        assert_eq!(
            resolve_odrl_conflict(&[], &[]),
            PolicyCompositionDecision::NoApplicableRule
        );
    }

    #[test]
    fn permission_without_prohibition_remains_permission() {
        assert_eq!(
            resolve_odrl_conflict(&[PolicyRuleEffect::Permission], &[]),
            PolicyCompositionDecision::Permission
        );
    }

    #[test]
    fn prohibition_without_permission_remains_prohibition() {
        assert_eq!(
            resolve_odrl_conflict(&[PolicyRuleEffect::Prohibition], &[]),
            PolicyCompositionDecision::Prohibition
        );
    }

    #[test]
    fn missing_conflict_strategy_defaults_to_invalid() {
        assert_eq!(
            resolve_odrl_conflict(
                &[PolicyRuleEffect::Permission, PolicyRuleEffect::Prohibition],
                &[]
            ),
            PolicyCompositionDecision::InvalidPolicy
        );
    }

    #[test]
    fn explicit_invalid_strategy_invalidates_conflicting_policy() {
        assert_eq!(
            resolve_odrl_conflict(
                &[PolicyRuleEffect::Permission, PolicyRuleEffect::Prohibition],
                &[OdrConflictStrategy::Invalid]
            ),
            PolicyCompositionDecision::InvalidPolicy
        );
    }

    #[test]
    fn perm_strategy_allows_permission_to_override_prohibition() {
        assert_eq!(
            resolve_odrl_conflict(
                &[PolicyRuleEffect::Permission, PolicyRuleEffect::Prohibition],
                &[OdrConflictStrategy::Perm]
            ),
            PolicyCompositionDecision::Permission
        );
    }

    #[test]
    fn prohibit_strategy_allows_prohibition_to_override_permission() {
        assert_eq!(
            resolve_odrl_conflict(
                &[PolicyRuleEffect::Permission, PolicyRuleEffect::Prohibition],
                &[OdrConflictStrategy::Prohibit]
            ),
            PolicyCompositionDecision::Prohibition
        );
    }

    #[test]
    fn multiple_strategy_values_in_a_conflict_invalidate_policy() {
        assert_eq!(
            resolve_odrl_conflict(
                &[PolicyRuleEffect::Permission, PolicyRuleEffect::Prohibition],
                &[OdrConflictStrategy::Perm, OdrConflictStrategy::Prohibit]
            ),
            PolicyCompositionDecision::InvalidPolicy
        );
    }

    #[test]
    fn duplicate_effects_do_not_create_a_false_conflict() {
        assert_eq!(
            resolve_odrl_conflict(
                &[PolicyRuleEffect::Permission, PolicyRuleEffect::Permission],
                &[OdrConflictStrategy::Invalid]
            ),
            PolicyCompositionDecision::Permission
        );
    }
}

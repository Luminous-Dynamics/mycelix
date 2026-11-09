use creator_asset_passport_core::policy::{
    OdrConflictStrategy, PolicyCompositionDecision, PolicyRuleEffect, resolve_odrl_conflict,
};

#[test]
fn conflicting_rules_default_to_invalid_when_strategy_is_missing() {
    assert_eq!(
        resolve_odrl_conflict(
            &[PolicyRuleEffect::Permission, PolicyRuleEffect::Prohibition],
            &[],
        ),
        PolicyCompositionDecision::InvalidPolicy
    );
}

#[test]
fn explicit_perm_strategy_resolves_conflict_to_permission() {
    assert_eq!(
        resolve_odrl_conflict(
            &[PolicyRuleEffect::Permission, PolicyRuleEffect::Prohibition],
            &[OdrConflictStrategy::Perm],
        ),
        PolicyCompositionDecision::Permission
    );
}

#[test]
fn explicit_prohibit_strategy_resolves_conflict_to_prohibition() {
    assert_eq!(
        resolve_odrl_conflict(
            &[PolicyRuleEffect::Permission, PolicyRuleEffect::Prohibition],
            &[OdrConflictStrategy::Prohibit],
        ),
        PolicyCompositionDecision::Prohibition
    );
}

#[test]
fn multiple_strategy_values_invalidate_a_conflicting_composition() {
    assert_eq!(
        resolve_odrl_conflict(
            &[PolicyRuleEffect::Permission, PolicyRuleEffect::Prohibition],
            &[OdrConflictStrategy::Perm, OdrConflictStrategy::Prohibit],
        ),
        PolicyCompositionDecision::InvalidPolicy
    );
}

#[test]
fn non_conflicting_rules_do_not_apply_conflict_precedence() {
    assert_eq!(
        resolve_odrl_conflict(
            &[PolicyRuleEffect::Permission],
            &[OdrConflictStrategy::Invalid],
        ),
        PolicyCompositionDecision::Permission
    );
    assert_eq!(
        resolve_odrl_conflict(
            &[PolicyRuleEffect::Prohibition],
            &[OdrConflictStrategy::Perm],
        ),
        PolicyCompositionDecision::Prohibition
    );
}

#[test]
fn empty_effect_set_does_not_authorize_or_deny() {
    assert_eq!(
        resolve_odrl_conflict(&[], &[OdrConflictStrategy::Perm]),
        PolicyCompositionDecision::NoApplicableRule
    );
}

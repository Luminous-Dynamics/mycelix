//! Economic Fabric V1 runtime-neutral conformance reference.
//! This module intentionally contains no ledger/network/financial authority.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Observed,
    Evidenced,
    Valuated,
    Entitled,
    Authorized,
    Submitted,
    SettlementEligible,
    Settled,
    Disputed,
    Reversed,
    Corrected,
    Indeterminate,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EconomicProfile {
    Generic,
    IntegralItc,
    Valueflows,
    MutualCredit,
    Tend,
    AccountingProjection,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TransitionContext {
    pub profile: EconomicProfile,
    pub source_evidence_current: bool,
    pub policy_version: Option<&'static str>,
    pub authority: Option<&'static str>,
    pub dispute_resolved: bool,
    pub claim_ceiling_preserved: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decision {
    Accepted,
    Rejected,
    DuplicateIdempotent,
    Indeterminate,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Event {
    pub id: &'static str,
    pub origin: &'static str,
    pub unit: &'static str,
    pub evidence: &'static str,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Settlement {
    pub logical_id: &'static str,
    pub source_event: &'static str,
    pub unit: &'static str,
    pub authorization: Option<&'static str>,
    pub attempt: &'static str,
}

pub fn can_transition(from: State, to: State) -> bool {
    matches!(
        (from, to),
        (State::Observed, State::Evidenced)
            | (State::Evidenced, State::Valuated)
            | (State::Valuated, State::Entitled)
            | (State::Entitled, State::Authorized)
            | (State::Authorized, State::Submitted)
            | (State::Submitted, State::SettlementEligible)
            | (State::Submitted, State::Indeterminate)
            | (State::SettlementEligible, State::Settled)
            | (State::Settled, State::Reversed)
            | (State::Settled, State::Corrected)
            | (State::Submitted, State::Disputed)
            | (State::Disputed, State::SettlementEligible)
            | (State::Disputed, State::Reversed)
    )
}

pub fn can_enter_settlement(current_state: State, ctx: &TransitionContext) -> bool {
    current_state == State::SettlementEligible
        && ctx.source_evidence_current
        && ctx.policy_version.is_some()
        && ctx.authority.is_some()
        && ctx.dispute_resolved
        && ctx.claim_ceiling_preserved
}

pub fn profile_allows_transition(
    profile: EconomicProfile,
    from: State,
    to: State,
) -> bool {
    match profile {
        EconomicProfile::IntegralItc => !matches!(
            (from, to),
            (State::Evidenced, State::Entitled)
        ),
        EconomicProfile::Valueflows => !matches!(
            (from, to),
            (State::Observed, State::Entitled)
        ),
        EconomicProfile::AccountingProjection => !matches!(
            (from, to),
            (State::Observed, State::Settled)
        ),
        EconomicProfile::Generic
        | EconomicProfile::MutualCredit
        | EconomicProfile::Tend => true,
    } && can_transition(from, to)
}

pub fn validate_settlement(
    s: &Settlement,
    current_state: State,
    recipient_accepted: bool,
    authorization_current: bool,
    already_settled: bool,
) -> Decision {
    if current_state == State::Disputed || !authorization_current || s.authorization.is_none() {
        return Decision::Rejected;
    }
    if already_settled {
        return Decision::DuplicateIdempotent;
    }
    if current_state == State::Indeterminate && !recipient_accepted {
        return Decision::Indeterminate;
    }
    if !recipient_accepted || current_state != State::SettlementEligible {
        return Decision::Rejected;
    }
    Decision::Accepted
}

#[cfg(test)]
mod tests {
    use super::*;

    fn context(profile: EconomicProfile) -> TransitionContext {
        TransitionContext {
            profile,
            source_evidence_current: true,
            policy_version: Some("P1"),
            authority: Some("AUTH-1"),
            dispute_resolved: true,
            claim_ceiling_preserved: true,
        }
    }

    #[test]
    fn transition_shortcuts_are_rejected() {
        assert!(!can_transition(State::Observed, State::Entitled));
        assert!(!can_transition(State::Evidenced, State::Settled));
        assert!(!can_transition(State::Valuated, State::Settled));
        assert!(!can_transition(State::Entitled, State::Settled));
        assert!(!can_transition(State::Submitted, State::Settled));
    }

    #[test]
    fn profile_guards_preserve_integral_itc_boundary() {
        assert!(!profile_allows_transition(
            EconomicProfile::IntegralItc,
            State::Evidenced,
            State::Entitled
        ));
        assert!(profile_allows_transition(
            EconomicProfile::IntegralItc,
            State::Valuated,
            State::Entitled
        ));
    }

    #[test]
    fn profile_guards_keep_accounting_derived() {
        assert!(!profile_allows_transition(
            EconomicProfile::AccountingProjection,
            State::Observed,
            State::Settled
        ));
    }

    #[test]
    fn valueflows_does_not_promote_observation_directly_to_entitlement() {
        assert!(!profile_allows_transition(
            EconomicProfile::Valueflows,
            State::Observed,
            State::Entitled
        ));
    }

    #[test]
    fn settlement_requires_policy_authority_and_current_evidence() {
        assert!(can_enter_settlement(State::SettlementEligible, &context(EconomicProfile::IntegralItc)));

        let mut no_authority = context(EconomicProfile::IntegralItc);
        no_authority.authority = None;
        assert!(!can_enter_settlement(State::SettlementEligible, &no_authority));

        let mut stale = context(EconomicProfile::Generic);
        stale.source_evidence_current = false;
        assert!(!can_enter_settlement(State::SettlementEligible, &stale));

        let mut widened = context(EconomicProfile::Generic);
        widened.claim_ceiling_preserved = false;
        assert!(!can_enter_settlement(State::SettlementEligible, &widened));
    }

    #[test]
    fn settlement_requires_recipient_acceptance() {
        let s = Settlement {
            logical_id: "L1",
            source_event: "E1",
            unit: "hour",
            authorization: Some("AUTH-1"),
            attempt: "A1",
        };
        assert_eq!(
            validate_settlement(&s, State::SettlementEligible, false, true, false),
            Decision::Rejected
        );
        assert_eq!(
            validate_settlement(&s, State::SettlementEligible, true, true, false),
            Decision::Accepted
        );
    }

    #[test]
    fn duplicate_logical_settlement_is_idempotent() {
        let s = Settlement {
            logical_id: "L1",
            source_event: "E1",
            unit: "hour",
            authorization: Some("AUTH-1"),
            attempt: "A2",
        };
        assert_eq!(
            validate_settlement(&s, State::SettlementEligible, true, true, true),
            Decision::DuplicateIdempotent
        );
    }

    #[test]
    fn dispute_requires_explicit_resolution_before_eligibility() {
        let unresolved = TransitionContext {
            dispute_resolved: false,
            ..context(EconomicProfile::IntegralItc)
        };
        assert!(!can_enter_settlement(State::Disputed, &unresolved));
        assert!(can_transition(State::Disputed, State::SettlementEligible));
        assert!(can_enter_settlement(
            State::SettlementEligible,
            &context(EconomicProfile::IntegralItc)
        ));
    }
}

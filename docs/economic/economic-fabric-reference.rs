//! Economic Fabric V1 runtime-neutral conformance reference.
//! This module intentionally contains no ledger/network/financial authority.

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Observed, Evidenced, Valuated, Entitled, Authorized, Submitted,
    Settled, Disputed, Reversed, Corrected, Indeterminate,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Decision {
    Accepted, Rejected, DuplicateIdempotent, Indeterminate,
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
    matches!((from, to),
        (State::Observed, State::Evidenced) |
        (State::Evidenced, State::Valuated) |
        (State::Valuated, State::Entitled) |
        (State::Entitled, State::Authorized) |
        (State::Authorized, State::Submitted) |
        (State::Submitted, State::Settled) |
        (State::Submitted, State::Indeterminate) |
        (State::Settled, State::Reversed) |
        (State::Settled, State::Corrected) |
        (State::Submitted, State::Disputed) |
        (State::Disputed, State::Settled) |
        (State::Disputed, State::Reversed) |
        (State::Indeterminate, State::Settled)
    )
}

pub fn validate_settlement(s: &Settlement, current_state: State, recipient_accepted: bool, authorization_current: bool, already_settled: bool) -> Decision {
    if current_state == State::Disputed || !authorization_current || s.authorization.is_none() {
        return Decision::Rejected;
    }
    if already_settled {
        return Decision::DuplicateIdempotent;
    }
    if current_state == State::Indeterminate && !recipient_accepted {
        return Decision::Indeterminate;
    }
    if !recipient_accepted || current_state != State::Submitted {
        return Decision::Rejected;
    }
    Decision::Accepted
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test] fn transition_shortcuts_are_rejected() {
        assert!(!can_transition(State::Observed, State::Entitled));
        assert!(!can_transition(State::Evidenced, State::Settled));
        assert!(!can_transition(State::Valuated, State::Settled));
        assert!(!can_transition(State::Entitled, State::Settled));
    }

    #[test] fn settlement_requires_authorization_and_acceptance() {
        let s = Settlement { logical_id:"L1", source_event:"E1", unit:"hour", authorization:None, attempt:"A1" };
        assert_eq!(validate_settlement(&s, State::Submitted, true, true, false), Decision::Rejected);
        let s = Settlement { authorization:Some("AUTH-1"), ..s };
        assert_eq!(validate_settlement(&s, State::Submitted, false, true, false), Decision::Rejected);
        assert_eq!(validate_settlement(&s, State::Submitted, true, true, false), Decision::Accepted);
    }

    #[test] fn duplicate_logical_settlement_is_idempotent() {
        let s = Settlement { logical_id:"L1", source_event:"E1", unit:"hour", authorization:Some("AUTH-1"), attempt:"A2" };
        assert_eq!(validate_settlement(&s, State::Submitted, true, true, true), Decision::DuplicateIdempotent);
    }

    #[test] fn disputed_and_indeterminate_do_not_silently_settle() {
        let s = Settlement { logical_id:"L1", source_event:"E1", unit:"hour", authorization:Some("AUTH-1"), attempt:"A1" };
        assert_eq!(validate_settlement(&s, State::Disputed, true, true, false), Decision::Rejected);
        assert_eq!(validate_settlement(&s, State::Indeterminate, false, true, false), Decision::Indeterminate);
    }
}
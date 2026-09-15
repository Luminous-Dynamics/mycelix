#![forbid(unsafe_code)]
//! ECON-004 exact TEND reciprocity policy.
//!
//! TEND is time-denominated, signed zero-sum mutual credit. The accounting unit in
//! this policy is the integer minute so sub-hour exchanges remain exact while the
//! display invariant remains 1 TEND = 1 hour.

use mycelix_economic_policy::{evaluate, EconomicEffect, EconomicLane, FirewallDecision};

pub const MINUTES_PER_TEND: i64 = 60;
pub const MAX_CANONICAL_CREDIT_TEND: i64 = 120;
pub const MAX_CANONICAL_CREDIT_MINUTES: i64 = MAX_CANONICAL_CREDIT_TEND * MINUTES_PER_TEND;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TendCreditTier {
    Normal,
    Elevated,
    High,
    Emergency,
}

impl TendCreditTier {
    pub const fn limit_tend(self) -> i64 {
        match self {
            Self::Normal => 40,
            Self::Elevated => 60,
            Self::High => 80,
            Self::Emergency => 120,
        }
    }

    pub const fn limit_minutes(self) -> i64 {
        self.limit_tend() * MINUTES_PER_TEND
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RelationshipKind {
    Reciprocity,
    Care,
    Gift,
}

/// Whether receiving under this relationship creates TEND repayment liability.
pub const fn creates_recipient_tend_liability(kind: RelationshipKind) -> bool {
    matches!(kind, RelationshipKind::Reciprocity)
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TendDelta {
    pub provider_minutes: i64,
    pub receiver_minutes: i64,
}

impl TendDelta {
    pub const fn is_zero_sum(self) -> bool {
        self.provider_minutes + self.receiver_minutes == 0
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct TendPostBalances {
    pub provider_minutes: i64,
    pub receiver_minutes: i64,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TendPolicyViolation {
    ZeroDuration,
    DurationTooLarge,
    InvalidCreditLimit,
    ArithmeticOverflow,
    ProviderCreditLimitExceeded,
    ReceiverCreditLimitExceeded,
    ConservationFailure,
}

/// Exact exchange delta for a positive number of service minutes.
pub fn exchange_delta(service_minutes: u32) -> Result<TendDelta, TendPolicyViolation> {
    if service_minutes == 0 {
        return Err(TendPolicyViolation::ZeroDuration);
    }
    let minutes = i64::from(service_minutes);
    if minutes > MAX_CANONICAL_CREDIT_MINUTES {
        return Err(TendPolicyViolation::DurationTooLarge);
    }
    let delta = TendDelta {
        provider_minutes: minutes,
        receiver_minutes: -minutes,
    };
    debug_assert!(delta.is_zero_sum());
    Ok(delta)
}

/// Apply one bilateral reciprocity exchange while preserving total credit exactly.
pub fn apply_exchange(
    provider_before: i64,
    receiver_before: i64,
    service_minutes: u32,
    credit_limit_tend: i64,
) -> Result<TendPostBalances, TendPolicyViolation> {
    if credit_limit_tend <= 0 || credit_limit_tend > MAX_CANONICAL_CREDIT_TEND {
        return Err(TendPolicyViolation::InvalidCreditLimit);
    }
    let limit_minutes = credit_limit_tend
        .checked_mul(MINUTES_PER_TEND)
        .ok_or(TendPolicyViolation::ArithmeticOverflow)?;
    let delta = exchange_delta(service_minutes)?;
    let provider_after = provider_before
        .checked_add(delta.provider_minutes)
        .ok_or(TendPolicyViolation::ArithmeticOverflow)?;
    let receiver_after = receiver_before
        .checked_add(delta.receiver_minutes)
        .ok_or(TendPolicyViolation::ArithmeticOverflow)?;

    if !within_limit(provider_after, limit_minutes) {
        return Err(TendPolicyViolation::ProviderCreditLimitExceeded);
    }
    if !within_limit(receiver_after, limit_minutes) {
        return Err(TendPolicyViolation::ReceiverCreditLimitExceeded);
    }

    let before_total = i128::from(provider_before) + i128::from(receiver_before);
    let after_total = i128::from(provider_after) + i128::from(receiver_after);
    if before_total != after_total {
        return Err(TendPolicyViolation::ConservationFailure);
    }

    Ok(TendPostBalances {
        provider_minutes: provider_after,
        receiver_minutes: receiver_after,
    })
}

const fn within_limit(value: i64, limit: i64) -> bool {
    value >= -limit && value <= limit
}

/// Canonical TEND never applies timed balance decay.
pub const fn tend_demurrage_is_forbidden() -> bool {
    matches!(
        evaluate(EconomicEffect::ApplyDemurrage {
            lane: EconomicLane::Tend,
        }),
        FirewallDecision::Forbidden
    )
}

/// Individual TEND balances are not automatically cash-convertible to SAP.
pub const fn automatic_tend_to_sap_cashout_is_forbidden() -> bool {
    matches!(
        evaluate(EconomicEffect::Convert {
            from: EconomicLane::Tend,
            to: EconomicLane::Sap,
        }),
        FirewallDecision::Forbidden
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn one_tend_is_exactly_one_hour() {
        assert_eq!(MINUTES_PER_TEND, 60);
        assert_eq!(TendCreditTier::Normal.limit_minutes(), 40 * 60);
    }

    #[test]
    fn sub_hour_exchange_remains_exact() {
        let delta = exchange_delta(15).unwrap();
        assert_eq!(delta.provider_minutes, 15);
        assert_eq!(delta.receiver_minutes, -15);
        assert!(delta.is_zero_sum());
    }

    #[test]
    fn bilateral_exchange_preserves_total_balance() {
        let result = apply_exchange(30, -30, 45, 40).unwrap();
        assert_eq!(result.provider_minutes, 75);
        assert_eq!(result.receiver_minutes, -75);
        assert_eq!(result.provider_minutes + result.receiver_minutes, 0);
    }

    #[test]
    fn conservation_holds_even_when_pair_total_is_nonzero() {
        let result = apply_exchange(120, 60, 30, 40).unwrap();
        assert_eq!(120 + 60, result.provider_minutes + result.receiver_minutes);
    }

    #[test]
    fn zero_duration_fails_closed() {
        assert_eq!(exchange_delta(0), Err(TendPolicyViolation::ZeroDuration));
    }

    #[test]
    fn canonical_credit_tiers_are_bounded_at_120_tend() {
        assert_eq!(TendCreditTier::Normal.limit_tend(), 40);
        assert_eq!(TendCreditTier::Elevated.limit_tend(), 60);
        assert_eq!(TendCreditTier::High.limit_tend(), 80);
        assert_eq!(TendCreditTier::Emergency.limit_tend(), 120);
        for tier in [
            TendCreditTier::Normal,
            TendCreditTier::Elevated,
            TendCreditTier::High,
            TendCreditTier::Emergency,
        ] {
            assert!(tier.limit_tend() <= MAX_CANONICAL_CREDIT_TEND);
        }
    }

    #[test]
    fn limit_overrun_fails_without_partial_application() {
        let almost_limit = 40 * 60 - 10;
        assert_eq!(
            apply_exchange(almost_limit, -almost_limit, 15, 40),
            Err(TendPolicyViolation::ProviderCreditLimitExceeded)
        );
    }

    #[test]
    fn need_and_gift_do_not_create_tend_debt() {
        assert!(creates_recipient_tend_liability(RelationshipKind::Reciprocity));
        assert!(!creates_recipient_tend_liability(RelationshipKind::Care));
        assert!(!creates_recipient_tend_liability(RelationshipKind::Gift));
    }

    #[test]
    fn canonical_tend_has_no_demurrage() {
        assert!(tend_demurrage_is_forbidden());
    }

    #[test]
    fn tend_is_not_automatically_cashable_into_sap() {
        assert!(automatic_tend_to_sap_cashout_is_forbidden());
    }
}

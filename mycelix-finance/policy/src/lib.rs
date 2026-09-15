#![forbid(unsafe_code)]
//! ECON-002 constitutional conversion/coupling firewall.
//!
//! This crate intentionally has no external dependencies and no authority to move value.
//! It answers only whether a proposed cross-lane economic effect is constitutionally
//! admissible under ECON-001. Runtime zomes must be wired to this policy in a later tranche.

/// Canonical foundational economic lanes.
#[derive(Clone, Copy, Debug, PartialEq, Eq, Hash)]
pub enum EconomicLane {
    /// Transferable settlement/accounting/exchange value.
    Sap,
    /// Signed zero-sum reciprocity / mutual-credit ledger.
    Tend,
    /// Non-transferable contextual standing / credential evidence.
    Mycel,
}

impl EconomicLane {
    pub const ALL: [Self; 3] = [Self::Sap, Self::Tend, Self::Mycel];

    pub const fn symbol(self) -> &'static str {
        match self {
            Self::Sap => "SAP",
            Self::Tend => "TEND",
            Self::Mycel => "MYCEL",
        }
    }
}

/// Economic operation submitted to the firewall.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum EconomicEffect {
    /// Transfer or ledger movement inside one lane.
    TransferWithin { lane: EconomicLane },
    /// Convert one foundational lane directly into another.
    Convert {
        from: EconomicLane,
        to: EconomicLane,
    },
    /// Apply balance decay/demurrage to a lane.
    ApplyDemurrage { lane: EconomicLane },
    /// Automatically create standing from SAP activity, holdings, payments, or gifts.
    AutomaticStandingFromSap,
    /// Automatically create standing from TEND balance, exchange volume, or ratings.
    AutomaticStandingFromTend,
    /// Treat an economic event merely as evidence that may be independently reviewed
    /// for a bounded domain credential. This operation does not itself grant standing.
    SubmitEconomicEventAsCredentialEvidence { source: EconomicLane },
    /// Make fundamental civic power depend automatically on a lane balance/score.
    FundamentalCivicPowerFrom { source: EconomicLane },
    /// Use independently reviewed MYCEL/domain evidence for bounded role eligibility.
    BoundedRoleEligibilityFromMycel,
}

/// Firewall disposition. Only `Allowed` may be executed without another authority step.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum FirewallDecision {
    /// Semantically compatible with the lane and may proceed subject to ordinary checks.
    Allowed,
    /// Input may be considered as evidence but a separate review/qualification authority
    /// must decide the resulting credential/role. This is never an automatic conversion.
    RequiresIndependentReview,
    /// Constitutionally forbidden cross-lane shortcut.
    Forbidden,
}

/// Evaluate a proposed effect against the ECON-001 lane constitution.
///
/// This function is deliberately total and fail-closed: every enum variant has an explicit
/// disposition and no unknown/default branch exists.
pub const fn evaluate(effect: EconomicEffect) -> FirewallDecision {
    match effect {
        EconomicEffect::TransferWithin {
            lane: EconomicLane::Sap,
        }
        | EconomicEffect::TransferWithin {
            lane: EconomicLane::Tend,
        } => FirewallDecision::Allowed,
        EconomicEffect::TransferWithin {
            lane: EconomicLane::Mycel,
        } => FirewallDecision::Forbidden,

        EconomicEffect::Convert { from, to } => {
            if same_lane(from, to) {
                // A no-op/same-lane normalization is not a cross-lane conversion.
                FirewallDecision::Allowed
            } else {
                FirewallDecision::Forbidden
            }
        }

        EconomicEffect::ApplyDemurrage {
            lane: EconomicLane::Sap,
        } => FirewallDecision::Allowed,
        EconomicEffect::ApplyDemurrage {
            lane: EconomicLane::Tend,
        }
        | EconomicEffect::ApplyDemurrage {
            lane: EconomicLane::Mycel,
        } => FirewallDecision::Forbidden,

        EconomicEffect::AutomaticStandingFromSap
        | EconomicEffect::AutomaticStandingFromTend => FirewallDecision::Forbidden,

        EconomicEffect::SubmitEconomicEventAsCredentialEvidence { .. } => {
            FirewallDecision::RequiresIndependentReview
        }

        EconomicEffect::FundamentalCivicPowerFrom { .. } => FirewallDecision::Forbidden,

        EconomicEffect::BoundedRoleEligibilityFromMycel => {
            FirewallDecision::RequiresIndependentReview
        }
    }
}

const fn same_lane(a: EconomicLane, b: EconomicLane) -> bool {
    matches!(
        (a, b),
        (EconomicLane::Sap, EconomicLane::Sap)
            | (EconomicLane::Tend, EconomicLane::Tend)
            | (EconomicLane::Mycel, EconomicLane::Mycel)
    )
}

/// Convenience predicate for callers that may only proceed on an immediately admissible effect.
pub const fn is_immediately_allowed(effect: EconomicEffect) -> bool {
    matches!(evaluate(effect), FirewallDecision::Allowed)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn canonical_symbols_are_unique_and_stable() {
        assert_eq!(EconomicLane::Sap.symbol(), "SAP");
        assert_eq!(EconomicLane::Tend.symbol(), "TEND");
        assert_eq!(EconomicLane::Mycel.symbol(), "MYCEL");
        assert_ne!(EconomicLane::Sap.symbol(), EconomicLane::Tend.symbol());
        assert_ne!(EconomicLane::Sap.symbol(), EconomicLane::Mycel.symbol());
        assert_ne!(EconomicLane::Tend.symbol(), EconomicLane::Mycel.symbol());
    }

    #[test]
    fn all_six_cross_lane_conversions_are_forbidden() {
        for from in EconomicLane::ALL {
            for to in EconomicLane::ALL {
                let decision = evaluate(EconomicEffect::Convert { from, to });
                if from == to {
                    assert_eq!(decision, FirewallDecision::Allowed);
                } else {
                    assert_eq!(
                        decision,
                        FirewallDecision::Forbidden,
                        "{} -> {} must be forbidden",
                        from.symbol(),
                        to.symbol()
                    );
                }
            }
        }
    }

    #[test]
    fn only_sap_and_tend_have_direct_transfer_semantics() {
        assert_eq!(
            evaluate(EconomicEffect::TransferWithin {
                lane: EconomicLane::Sap,
            }),
            FirewallDecision::Allowed
        );
        assert_eq!(
            evaluate(EconomicEffect::TransferWithin {
                lane: EconomicLane::Tend,
            }),
            FirewallDecision::Allowed
        );
        assert_eq!(
            evaluate(EconomicEffect::TransferWithin {
                lane: EconomicLane::Mycel,
            }),
            FirewallDecision::Forbidden
        );
    }

    #[test]
    fn demurrage_is_sap_only() {
        assert_eq!(
            evaluate(EconomicEffect::ApplyDemurrage {
                lane: EconomicLane::Sap,
            }),
            FirewallDecision::Allowed
        );
        assert_eq!(
            evaluate(EconomicEffect::ApplyDemurrage {
                lane: EconomicLane::Tend,
            }),
            FirewallDecision::Forbidden
        );
        assert_eq!(
            evaluate(EconomicEffect::ApplyDemurrage {
                lane: EconomicLane::Mycel,
            }),
            FirewallDecision::Forbidden
        );
    }

    #[test]
    fn economic_activity_cannot_automatically_mint_standing() {
        assert_eq!(
            evaluate(EconomicEffect::AutomaticStandingFromSap),
            FirewallDecision::Forbidden
        );
        assert_eq!(
            evaluate(EconomicEffect::AutomaticStandingFromTend),
            FirewallDecision::Forbidden
        );
    }

    #[test]
    fn economic_events_may_only_enter_standing_as_reviewable_evidence() {
        for source in [EconomicLane::Sap, EconomicLane::Tend] {
            assert_eq!(
                evaluate(EconomicEffect::SubmitEconomicEventAsCredentialEvidence { source }),
                FirewallDecision::RequiresIndependentReview
            );
        }
    }

    #[test]
    fn no_lane_balance_or_score_grants_fundamental_civic_power() {
        for source in EconomicLane::ALL {
            assert_eq!(
                evaluate(EconomicEffect::FundamentalCivicPowerFrom { source }),
                FirewallDecision::Forbidden
            );
        }
    }

    #[test]
    fn mycel_role_eligibility_is_not_automatic_authority() {
        assert_eq!(
            evaluate(EconomicEffect::BoundedRoleEligibilityFromMycel),
            FirewallDecision::RequiresIndependentReview
        );
        assert!(!is_immediately_allowed(
            EconomicEffect::BoundedRoleEligibilityFromMycel
        ));
    }
}

#![forbid(unsafe_code)]
//! ECON-006 typed economic claims policy.
//!
//! Financial and institutional instruments are represented as typed claims, rights,
//! obligations, pools, or contracts rather than new foundational currencies.

use mycelix_economic_policy::{evaluate, EconomicEffect, EconomicLane, FirewallDecision};
use mycelix_sap_policy::MICRO_SAP_PER_SAP;
use mycelix_tend_policy::MINUTES_PER_TEND;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClaimKind {
    Equity,
    Debt,
    Bond,
    Mortgage,
    Insurance,
    Escrow,
    RevenueShare,
    StewardshipRight,
    Subscription,
    Grant,
    Pension,
    InfrastructureRight,
    CapacityReservation,
    Other,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Transferability {
    Transferable,
    NonTransferable,
    ConsentRequired,
    AssignmentRestricted,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ClaimPerformance {
    /// A right/obligation with no monetary settlement amount.
    None,
    /// SAP-denominated settlement using existing SAP units.
    Sap { amount_micro_sap: u64 },
    /// A voluntary TEND reciprocity obligation measured in exact minutes.
    TendReciprocity { minutes: u32 },
    /// Quantity of a non-monetary resource/service/capacity.
    NonMonetary { quantity: u64, unit: String },
    /// External denomination. This does not create a Mycelix foundational currency.
    External {
        namespace: String,
        code: String,
        minor_units: u64,
    },
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, Default)]
pub struct ClaimRights {
    pub use_right: bool,
    pub income_right: bool,
    pub transfer_right: bool,
    pub exclusion_right: bool,
    pub stewardship_duty: bool,
}

impl ClaimRights {
    pub const fn has_any(self) -> bool {
        self.use_right
            || self.income_right
            || self.transfer_right
            || self.exclusion_right
            || self.stewardship_duty
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct TypedClaim {
    pub claim_id: String,
    pub issuer_ref: String,
    pub holder_ref: String,
    pub subject_ref: String,
    pub kind: ClaimKind,
    pub transferability: Transferability,
    pub rights: ClaimRights,
    pub performance: ClaimPerformance,
    pub evidence_refs: Vec<String>,
    pub created_at: u64,
    pub expires_at: Option<u64>,
    pub challenged: bool,
    pub cancelled: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClaimViolation {
    MissingClaimId,
    MissingIssuer,
    MissingHolder,
    MissingSubject,
    MissingEvidence,
    NoRightsOrPerformance,
    InvalidExpiry,
    ContradictoryTransferRight,
    MissingStewardshipDuty,
    InvalidSapAmount,
    InvalidTendMinutes,
    InvalidNonMonetaryQuantity,
    MissingNonMonetaryUnit,
    InvalidExternalAmount,
    MissingExternalNamespace,
    MissingExternalCode,
}

pub fn validate_claim(claim: &TypedClaim) -> Result<(), ClaimViolation> {
    if claim.claim_id.trim().is_empty() {
        return Err(ClaimViolation::MissingClaimId);
    }
    if claim.issuer_ref.trim().is_empty() {
        return Err(ClaimViolation::MissingIssuer);
    }
    if claim.holder_ref.trim().is_empty() {
        return Err(ClaimViolation::MissingHolder);
    }
    if claim.subject_ref.trim().is_empty() {
        return Err(ClaimViolation::MissingSubject);
    }
    if claim.evidence_refs.is_empty()
        || claim.evidence_refs.iter().any(|reference| reference.trim().is_empty())
    {
        return Err(ClaimViolation::MissingEvidence);
    }
    if let Some(expiry) = claim.expires_at {
        if expiry <= claim.created_at {
            return Err(ClaimViolation::InvalidExpiry);
        }
    }
    if matches!(claim.transferability, Transferability::NonTransferable)
        && claim.rights.transfer_right
    {
        return Err(ClaimViolation::ContradictoryTransferRight);
    }
    if matches!(claim.kind, ClaimKind::StewardshipRight) && !claim.rights.stewardship_duty {
        return Err(ClaimViolation::MissingStewardshipDuty);
    }
    validate_performance(&claim.performance)?;
    if !claim.rights.has_any() && matches!(claim.performance, ClaimPerformance::None) {
        return Err(ClaimViolation::NoRightsOrPerformance);
    }
    Ok(())
}

fn validate_performance(performance: &ClaimPerformance) -> Result<(), ClaimViolation> {
    match performance {
        ClaimPerformance::None => Ok(()),
        ClaimPerformance::Sap { amount_micro_sap } => {
            if *amount_micro_sap == 0 {
                Err(ClaimViolation::InvalidSapAmount)
            } else {
                Ok(())
            }
        }
        ClaimPerformance::TendReciprocity { minutes } => {
            if *minutes == 0 {
                Err(ClaimViolation::InvalidTendMinutes)
            } else {
                Ok(())
            }
        }
        ClaimPerformance::NonMonetary { quantity, unit } => {
            if *quantity == 0 {
                Err(ClaimViolation::InvalidNonMonetaryQuantity)
            } else if unit.trim().is_empty() {
                Err(ClaimViolation::MissingNonMonetaryUnit)
            } else {
                Ok(())
            }
        }
        ClaimPerformance::External {
            namespace,
            code,
            minor_units,
        } => {
            if namespace.trim().is_empty() {
                Err(ClaimViolation::MissingExternalNamespace)
            } else if code.trim().is_empty() {
                Err(ClaimViolation::MissingExternalCode)
            } else if *minor_units == 0 {
                Err(ClaimViolation::InvalidExternalAmount)
            } else {
                Ok(())
            }
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClaimState {
    Pending,
    Active,
    Challenged,
    Cancelled,
    Expired,
    StructurallyInvalid,
}

pub fn claim_state(claim: &TypedClaim, now: u64) -> ClaimState {
    if validate_claim(claim).is_err() {
        return ClaimState::StructurallyInvalid;
    }
    if claim.cancelled {
        return ClaimState::Cancelled;
    }
    if claim.challenged {
        return ClaimState::Challenged;
    }
    if now < claim.created_at {
        return ClaimState::Pending;
    }
    if claim.expires_at.is_some_and(|expiry| now >= expiry) {
        return ClaimState::Expired;
    }
    ClaimState::Active
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ClaimLaneEffect {
    pub touches_sap: bool,
    pub touches_tend: bool,
    pub touches_mycel: bool,
    pub creates_foundational_currency: bool,
}

pub const fn lane_effect(performance: &ClaimPerformance) -> ClaimLaneEffect {
    match performance {
        ClaimPerformance::Sap { .. } => ClaimLaneEffect {
            touches_sap: true,
            touches_tend: false,
            touches_mycel: false,
            creates_foundational_currency: false,
        },
        ClaimPerformance::TendReciprocity { .. } => ClaimLaneEffect {
            touches_sap: false,
            touches_tend: true,
            touches_mycel: false,
            creates_foundational_currency: false,
        },
        ClaimPerformance::None
        | ClaimPerformance::NonMonetary { .. }
        | ClaimPerformance::External { .. } => ClaimLaneEffect {
            touches_sap: false,
            touches_tend: false,
            touches_mycel: false,
            creates_foundational_currency: false,
        },
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClaimAuthorityUse {
    ContractualOrPropertyRight,
    FundamentalCivicPower,
    HumanWorthRanking,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ClaimAuthorityDecision {
    GovernedByClaimTerms,
    Forbidden,
}

pub const fn authority_use(use_case: ClaimAuthorityUse) -> ClaimAuthorityDecision {
    match use_case {
        ClaimAuthorityUse::ContractualOrPropertyRight => ClaimAuthorityDecision::GovernedByClaimTerms,
        ClaimAuthorityUse::FundamentalCivicPower | ClaimAuthorityUse::HumanWorthRanking => {
            ClaimAuthorityDecision::Forbidden
        }
    }
}

pub const fn sap_performance_uses_canonical_scale() -> u64 {
    MICRO_SAP_PER_SAP
}

pub const fn tend_performance_uses_canonical_minutes() -> i64 {
    MINUTES_PER_TEND
}

/// A typed claim never authorizes hidden cross-lane conversion.
pub const fn sap_to_tend_conversion_remains_forbidden() -> bool {
    matches!(
        evaluate(EconomicEffect::Convert {
            from: EconomicLane::Sap,
            to: EconomicLane::Tend,
        }),
        FirewallDecision::Forbidden
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn base_claim(kind: ClaimKind, performance: ClaimPerformance) -> TypedClaim {
        TypedClaim {
            claim_id: "claim:1".into(),
            issuer_ref: "issuer:1".into(),
            holder_ref: "holder:1".into(),
            subject_ref: "subject:1".into(),
            kind,
            transferability: Transferability::AssignmentRestricted,
            rights: ClaimRights {
                use_right: true,
                ..ClaimRights::default()
            },
            performance,
            evidence_refs: vec!["evidence:1".into()],
            created_at: 100,
            expires_at: Some(200),
            challenged: false,
            cancelled: false,
        }
    }

    #[test]
    fn major_financial_instruments_fit_claim_model_without_new_currency() {
        for kind in [
            ClaimKind::Equity,
            ClaimKind::Debt,
            ClaimKind::Bond,
            ClaimKind::Mortgage,
            ClaimKind::Insurance,
            ClaimKind::Escrow,
            ClaimKind::RevenueShare,
            ClaimKind::Subscription,
            ClaimKind::Grant,
            ClaimKind::Pension,
            ClaimKind::InfrastructureRight,
            ClaimKind::CapacityReservation,
        ] {
            let claim = base_claim(
                kind,
                ClaimPerformance::Sap {
                    amount_micro_sap: MICRO_SAP_PER_SAP,
                },
            );
            assert_eq!(validate_claim(&claim), Ok(()));
            assert!(!lane_effect(&claim.performance).creates_foundational_currency);
        }
    }

    #[test]
    fn sap_claim_touches_only_sap_lane() {
        let effect = lane_effect(&ClaimPerformance::Sap {
            amount_micro_sap: MICRO_SAP_PER_SAP,
        });
        assert!(effect.touches_sap);
        assert!(!effect.touches_tend);
        assert!(!effect.touches_mycel);
        assert!(!effect.creates_foundational_currency);
    }

    #[test]
    fn tend_reciprocity_claim_does_not_become_sap_or_mycel() {
        let effect = lane_effect(&ClaimPerformance::TendReciprocity { minutes: 30 });
        assert!(!effect.touches_sap);
        assert!(effect.touches_tend);
        assert!(!effect.touches_mycel);
        assert!(!effect.creates_foundational_currency);
        assert!(sap_to_tend_conversion_remains_forbidden());
    }

    #[test]
    fn external_denominations_do_not_become_foundational_mycelix_currencies() {
        let performance = ClaimPerformance::External {
            namespace: "iso4217".into(),
            code: "ZAR".into(),
            minor_units: 10_000,
        };
        assert_eq!(validate_performance(&performance), Ok(()));
        assert!(!lane_effect(&performance).creates_foundational_currency);
    }

    #[test]
    fn stewardship_claim_requires_stewardship_duty() {
        let mut claim = base_claim(ClaimKind::StewardshipRight, ClaimPerformance::None);
        assert_eq!(
            validate_claim(&claim),
            Err(ClaimViolation::MissingStewardshipDuty)
        );
        claim.rights.stewardship_duty = true;
        assert_eq!(validate_claim(&claim), Ok(()));
    }

    #[test]
    fn nontransferable_claim_cannot_advertise_transfer_right() {
        let mut claim = base_claim(ClaimKind::Grant, ClaimPerformance::None);
        claim.transferability = Transferability::NonTransferable;
        claim.rights.transfer_right = true;
        assert_eq!(
            validate_claim(&claim),
            Err(ClaimViolation::ContradictoryTransferRight)
        );
    }

    #[test]
    fn challenges_and_expiry_change_claim_state_without_rewriting_history() {
        let mut claim = base_claim(ClaimKind::Insurance, ClaimPerformance::None);
        assert_eq!(claim_state(&claim, 150), ClaimState::Active);
        claim.challenged = true;
        assert_eq!(claim_state(&claim, 150), ClaimState::Challenged);
        claim.challenged = false;
        assert_eq!(claim_state(&claim, 200), ClaimState::Expired);
    }

    #[test]
    fn claims_cannot_create_fundamental_civic_power_or_human_rank() {
        assert_eq!(
            authority_use(ClaimAuthorityUse::FundamentalCivicPower),
            ClaimAuthorityDecision::Forbidden
        );
        assert_eq!(
            authority_use(ClaimAuthorityUse::HumanWorthRanking),
            ClaimAuthorityDecision::Forbidden
        );
        assert_eq!(
            authority_use(ClaimAuthorityUse::ContractualOrPropertyRight),
            ClaimAuthorityDecision::GovernedByClaimTerms
        );
    }

    #[test]
    fn canonical_units_are_reused_not_reinvented() {
        assert_eq!(sap_performance_uses_canonical_scale(), 1_000_000);
        assert_eq!(tend_performance_uses_canonical_minutes(), 60);
    }
}

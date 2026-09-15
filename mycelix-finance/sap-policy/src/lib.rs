#![forbid(unsafe_code)]
//! ECON-003 SAP unit/balance/issuance semantic boundary.
//!
//! This crate is policy only. It does not move SAP, price collateral, redeem assets,
//! or decide reserve sufficiency. It distinguishes denomination from balances and
//! makes minting a separate, explicit authority event.

use mycelix_economic_policy::{evaluate, EconomicEffect, EconomicLane, FirewallDecision};

/// Canonical SAP wire symbol.
pub const SAP_SYMBOL: &str = "SAP";
/// Canonical integer scale used by current Finance wire/runtime surfaces.
pub const MICRO_SAP_PER_SAP: u64 = 1_000_000;

/// Immutable definition of the SAP unit of account.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SapUnitDefinition {
    pub symbol: &'static str,
    pub micro_units_per_unit: u64,
}

pub const SAP_UNIT: SapUnitDefinition = SapUnitDefinition {
    symbol: SAP_SYMBOL,
    micro_units_per_unit: MICRO_SAP_PER_SAP,
};

/// Balance-policy class. This classifies balance treatment, not the denomination.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SapBalanceClass {
    /// Ordinary user/organization SAP balance; may be demurrage eligible.
    Ordinary,
    /// Commons reserve whose policy may exempt it from balance decay.
    CommonsReserve,
    /// Explicitly authorized protected tranche (for example an expiring exemption).
    ProtectedExempt,
}

/// Whether balance demurrage may be applied under the canonical semantics.
///
/// This says nothing about the actual rate or whether governance has activated one.
pub const fn demurrage_semantically_permitted(class: SapBalanceClass) -> bool {
    matches!(class, SapBalanceClass::Ordinary)
}

/// Demurrage changes eligible balances, never the definition of one SAP.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct DemurrageSemanticEffect {
    pub balance_may_change: bool,
    pub unit_definition_changes: bool,
}

pub const fn demurrage_effect(class: SapBalanceClass) -> DemurrageSemanticEffect {
    DemurrageSemanticEffect {
        balance_may_change: demurrage_semantically_permitted(class),
        unit_definition_changes: false,
    }
}

/// Explicit bases on which a future SAP issuance request may be evaluated.
///
/// Presence of a variant is not proof that the backing is sufficient or authentic.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SapIssuanceBasis {
    VerifiedCollateral,
    RatifiedGovernanceAuthorization,
    ExplicitExternalValueSettlement,
    BootstrapDistribution,
}

/// Minimal semantic envelope for issuing new SAP.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SapIssuanceRequest<'a> {
    pub amount_micro_sap: u64,
    pub basis: SapIssuanceBasis,
    pub authority_ref: &'a str,
    pub evidence_ref: &'a str,
    /// Required only for external-value settlement. Ordinary checkout is not issuance.
    pub explicit_external_conversion_intent: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SapPolicyViolation {
    ZeroIssuance,
    MissingAuthorityReference,
    MissingEvidenceReference,
    MissingExplicitExternalConversionIntent,
}

/// Validate only the semantic envelope for issuance.
///
/// This does not validate signatures, collateral value, legal authority, reserves,
/// external payment finality, or oracle truth; those belong to qualified adapters.
pub fn validate_issuance(request: &SapIssuanceRequest<'_>) -> Result<(), SapPolicyViolation> {
    if request.amount_micro_sap == 0 {
        return Err(SapPolicyViolation::ZeroIssuance);
    }
    if request.authority_ref.trim().is_empty() {
        return Err(SapPolicyViolation::MissingAuthorityReference);
    }
    if request.evidence_ref.trim().is_empty() {
        return Err(SapPolicyViolation::MissingEvidenceReference);
    }
    if matches!(request.basis, SapIssuanceBasis::ExplicitExternalValueSettlement)
        && !request.explicit_external_conversion_intent
    {
        return Err(SapPolicyViolation::MissingExplicitExternalConversionIntent);
    }
    Ok(())
}

/// Economic effect of an ordinary checkout/payment request.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CheckoutEffect {
    /// Move already-existing SAP. The checkout itself creates no new SAP.
    TransferExistingSap,
}

pub const fn ordinary_checkout_effect() -> CheckoutEffect {
    CheckoutEffect::TransferExistingSap
}

/// Typed non-currency claims that SAP may settle or redeem against.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SapClaimKind {
    Energy,
    AgriculturalGoods,
    HousingUse,
    CarbonRestoration,
    Service,
    Other,
}

/// Semantic effect of settling SAP against a typed claim.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct SapClaimEffect {
    /// SAP settlement/redemption accounting may occur in the SAP lane.
    pub touches_sap: bool,
    /// The recipient receives a typed claim/right/service obligation.
    pub creates_typed_claim: bool,
    /// The operation never implicitly changes TEND mutual-credit balances.
    pub mutates_tend: bool,
    /// The operation never implicitly changes MYCEL standing.
    pub mutates_mycel: bool,
}

pub const fn settle_typed_claim(_kind: SapClaimKind) -> SapClaimEffect {
    SapClaimEffect {
        touches_sap: true,
        creates_typed_claim: true,
        mutates_tend: false,
        mutates_mycel: false,
    }
}

/// A SAP-denominated service claim is not a SAP->TEND conversion.
pub const fn service_claim_respects_conversion_firewall() -> bool {
    matches!(
        evaluate(EconomicEffect::Convert {
            from: EconomicLane::Sap,
            to: EconomicLane::Tend,
        }),
        FirewallDecision::Forbidden
    ) && !settle_typed_claim(SapClaimKind::Service).mutates_tend
}

#[cfg(test)]
mod tests {
    use super::*;

    fn valid_request(basis: SapIssuanceBasis) -> SapIssuanceRequest<'static> {
        SapIssuanceRequest {
            amount_micro_sap: MICRO_SAP_PER_SAP,
            basis,
            authority_ref: "authority:example",
            evidence_ref: "evidence:example",
            explicit_external_conversion_intent: true,
        }
    }

    #[test]
    fn canonical_unit_is_stable() {
        assert_eq!(SAP_UNIT.symbol, "SAP");
        assert_eq!(SAP_UNIT.micro_units_per_unit, 1_000_000);
        assert_eq!(SAP_UNIT, SAP_UNIT);
    }

    #[test]
    fn demurrage_is_balance_treatment_not_unit_redefinition() {
        for class in [
            SapBalanceClass::Ordinary,
            SapBalanceClass::CommonsReserve,
            SapBalanceClass::ProtectedExempt,
        ] {
            let effect = demurrage_effect(class);
            assert!(!effect.unit_definition_changes);
            assert_eq!(SAP_UNIT.symbol, "SAP");
            assert_eq!(SAP_UNIT.micro_units_per_unit, MICRO_SAP_PER_SAP);
        }
        assert!(demurrage_effect(SapBalanceClass::Ordinary).balance_may_change);
        assert!(!demurrage_effect(SapBalanceClass::CommonsReserve).balance_may_change);
        assert!(!demurrage_effect(SapBalanceClass::ProtectedExempt).balance_may_change);
    }

    #[test]
    fn ordinary_checkout_never_means_mint() {
        assert_eq!(
            ordinary_checkout_effect(),
            CheckoutEffect::TransferExistingSap
        );
    }

    #[test]
    fn issuance_requires_positive_amount_authority_and_evidence() {
        let mut request = valid_request(SapIssuanceBasis::VerifiedCollateral);
        assert_eq!(validate_issuance(&request), Ok(()));

        request.amount_micro_sap = 0;
        assert_eq!(
            validate_issuance(&request),
            Err(SapPolicyViolation::ZeroIssuance)
        );

        request = valid_request(SapIssuanceBasis::VerifiedCollateral);
        request.authority_ref = "";
        assert_eq!(
            validate_issuance(&request),
            Err(SapPolicyViolation::MissingAuthorityReference)
        );

        request = valid_request(SapIssuanceBasis::VerifiedCollateral);
        request.evidence_ref = "  ";
        assert_eq!(
            validate_issuance(&request),
            Err(SapPolicyViolation::MissingEvidenceReference)
        );
    }

    #[test]
    fn external_value_to_sap_requires_separate_explicit_intent() {
        let mut request = valid_request(SapIssuanceBasis::ExplicitExternalValueSettlement);
        request.explicit_external_conversion_intent = false;
        assert_eq!(
            validate_issuance(&request),
            Err(SapPolicyViolation::MissingExplicitExternalConversionIntent)
        );
        request.explicit_external_conversion_intent = true;
        assert_eq!(validate_issuance(&request), Ok(()));
    }

    #[test]
    fn non_external_issuance_does_not_reuse_external_conversion_flag_as_authority() {
        for basis in [
            SapIssuanceBasis::VerifiedCollateral,
            SapIssuanceBasis::RatifiedGovernanceAuthorization,
            SapIssuanceBasis::BootstrapDistribution,
        ] {
            let mut request = valid_request(basis);
            request.explicit_external_conversion_intent = false;
            assert_eq!(validate_issuance(&request), Ok(()));
        }
    }

    #[test]
    fn typed_claims_do_not_mutate_other_foundational_lanes() {
        for kind in [
            SapClaimKind::Energy,
            SapClaimKind::AgriculturalGoods,
            SapClaimKind::HousingUse,
            SapClaimKind::CarbonRestoration,
            SapClaimKind::Service,
            SapClaimKind::Other,
        ] {
            let effect = settle_typed_claim(kind);
            assert!(effect.touches_sap);
            assert!(effect.creates_typed_claim);
            assert!(!effect.mutates_tend);
            assert!(!effect.mutates_mycel);
        }
    }

    #[test]
    fn service_claim_is_not_tend_conversion() {
        assert!(service_claim_respects_conversion_firewall());
        assert_eq!(
            evaluate(EconomicEffect::Convert {
                from: EconomicLane::Sap,
                to: EconomicLane::Tend,
            }),
            FirewallDecision::Forbidden
        );
    }
}

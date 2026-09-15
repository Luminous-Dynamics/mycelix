#![forbid(unsafe_code)]
//! ECON-Q1 cross-kernel conformance capsule.
//!
//! This crate adds no economic semantics. It composes the frozen ECON-002..006 policy
//! surfaces and tests that their boundaries remain intact when used together.

#[cfg(test)]
mod tests {
    use mycelix_claims_policy::{
        authority_use, lane_effect, sap_performance_uses_canonical_scale,
        sap_to_tend_conversion_remains_forbidden, tend_performance_uses_canonical_minutes,
        validate_claim, ClaimAuthorityDecision, ClaimAuthorityUse, ClaimKind, ClaimPerformance,
        ClaimRights, Transferability, TypedClaim,
    };
    use mycelix_economic_policy::{
        evaluate, EconomicEffect, EconomicLane, FirewallDecision,
    };
    use mycelix_mycel_policy::{
        credential_state, economic_event_requires_review, evaluate_role_evidence, evaluate_use,
        BoundedRoleRequirement, CredentialState, MycelCredential, MycelUse, MycelUseDecision,
        RoleEvidenceDecision,
    };
    use mycelix_sap_policy::{
        ordinary_checkout_effect, service_claim_respects_conversion_firewall, settle_typed_claim,
        validate_issuance, CheckoutEffect, SapClaimKind, SapIssuanceBasis, SapIssuanceRequest,
        SapPolicyViolation, MICRO_SAP_PER_SAP,
    };
    use mycelix_tend_policy::{
        apply_exchange, creates_recipient_tend_liability, tend_demurrage_is_forbidden,
        RelationshipKind, TendCreditTier, MAX_CANONICAL_CREDIT_TEND, MINUTES_PER_TEND,
    };

    fn credential(domain: &str, capability: &str) -> MycelCredential {
        MycelCredential {
            subject_ref: "did:mycelix:alice".into(),
            domain: domain.into(),
            capability: capability.into(),
            issuer_ref: "authority:independent-review".into(),
            evidence_refs: vec!["evidence:economic-event:1".into()],
            issued_at: 100,
            expires_at: 200,
            challenged: false,
            revoked: false,
        }
    }

    fn claim(kind: ClaimKind, performance: ClaimPerformance) -> TypedClaim {
        let stewardship = matches!(kind, ClaimKind::StewardshipRight);
        TypedClaim {
            claim_id: "claim:q1".into(),
            issuer_ref: "issuer:q1".into(),
            holder_ref: "holder:q1".into(),
            subject_ref: "subject:q1".into(),
            kind,
            transferability: Transferability::AssignmentRestricted,
            rights: ClaimRights {
                use_right: true,
                stewardship_duty: stewardship,
                ..ClaimRights::default()
            },
            performance,
            evidence_refs: vec!["evidence:q1".into()],
            created_at: 100,
            expires_at: Some(200),
            challenged: false,
            cancelled: false,
        }
    }

    #[test]
    fn every_direct_cross_lane_conversion_fails_closed() {
        for from in EconomicLane::ALL {
            for to in EconomicLane::ALL {
                let decision = evaluate(EconomicEffect::Convert { from, to });
                if from == to {
                    assert_eq!(decision, FirewallDecision::Allowed);
                } else {
                    assert_eq!(decision, FirewallDecision::Forbidden);
                }
            }
        }
    }

    #[test]
    fn ordinary_sap_checkout_is_transfer_not_issuance_or_cross_lane_conversion() {
        assert_eq!(
            ordinary_checkout_effect(),
            CheckoutEffect::TransferExistingSap
        );
        let effect = settle_typed_claim(SapClaimKind::Service);
        assert!(effect.touches_sap);
        assert!(effect.creates_typed_claim);
        assert!(!effect.mutates_tend);
        assert!(!effect.mutates_mycel);
        assert!(service_claim_respects_conversion_firewall());
    }

    #[test]
    fn external_value_cannot_silently_become_sap() {
        let request = SapIssuanceRequest {
            amount_micro_sap: MICRO_SAP_PER_SAP,
            basis: SapIssuanceBasis::ExplicitExternalValueSettlement,
            authority_ref: "authority:q1",
            evidence_ref: "evidence:settlement:q1",
            explicit_external_conversion_intent: false,
        };
        assert_eq!(
            validate_issuance(&request),
            Err(SapPolicyViolation::MissingExplicitExternalConversionIntent)
        );
    }

    #[test]
    fn tend_subhour_sequences_preserve_exact_zero_sum_accounting() {
        let mut provider = 0_i64;
        let mut receiver = 0_i64;
        for minutes in [15_u32, 45, 30, 90] {
            let next = apply_exchange(provider, receiver, minutes, 40).unwrap();
            provider = next.provider_minutes;
            receiver = next.receiver_minutes;
            assert_eq!(provider + receiver, 0);
        }
        assert_eq!(provider, 180);
        assert_eq!(receiver, -180);
        assert_eq!(MINUTES_PER_TEND, 60);
    }

    #[test]
    fn tend_emergency_capacity_has_a_hard_declared_ceiling() {
        assert_eq!(TendCreditTier::Emergency.limit_tend(), 120);
        assert_eq!(MAX_CANONICAL_CREDIT_TEND, 120);
        assert!(TendCreditTier::Emergency.limit_tend() <= MAX_CANONICAL_CREDIT_TEND);
        assert!(tend_demurrage_is_forbidden());
    }

    #[test]
    fn care_and_gifts_cannot_create_reciprocity_debt() {
        assert!(creates_recipient_tend_liability(RelationshipKind::Reciprocity));
        assert!(!creates_recipient_tend_liability(RelationshipKind::Care));
        assert!(!creates_recipient_tend_liability(RelationshipKind::Gift));
    }

    #[test]
    fn economic_activity_can_enter_mycel_only_as_reviewable_evidence() {
        assert!(economic_event_requires_review(EconomicLane::Sap));
        assert!(economic_event_requires_review(EconomicLane::Tend));
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
    fn a_matching_mycel_credential_never_grants_a_role_by_itself() {
        let c = credential("mediation", "mediator");
        let requirement = BoundedRoleRequirement {
            domain: "mediation".into(),
            capability: "mediator".into(),
        };
        assert_eq!(credential_state(&c, 150), CredentialState::Active);
        assert_eq!(
            evaluate_role_evidence(&c, &requirement, 150),
            RoleEvidenceDecision::SupportsIndependentReview
        );
        assert_eq!(
            evaluate_use(MycelUse::BoundedRoleEvidence),
            MycelUseDecision::EvidenceOnly
        );
    }

    #[test]
    fn challenged_revoked_or_expired_credentials_fail_closed() {
        let mut challenged = credential("mediation", "mediator");
        challenged.challenged = true;
        assert_eq!(credential_state(&challenged, 150), CredentialState::Challenged);

        let mut revoked = credential("mediation", "mediator");
        revoked.revoked = true;
        assert_eq!(credential_state(&revoked, 150), CredentialState::Revoked);

        let expired = credential("mediation", "mediator");
        assert_eq!(credential_state(&expired, 200), CredentialState::Expired);
    }

    #[test]
    fn even_valid_mycel_cannot_be_laundered_into_economic_privilege() {
        for use_case in [
            MycelUse::AutomaticSapFeeDiscount,
            MycelUse::AutomaticTendCreditExpansion,
            MycelUse::AutomaticEconomicPayout,
        ] {
            assert_eq!(evaluate_use(use_case), MycelUseDecision::Forbidden);
        }
    }

    #[test]
    fn major_financial_instruments_remain_claims_not_currencies() {
        for kind in [
            ClaimKind::Equity,
            ClaimKind::Debt,
            ClaimKind::Bond,
            ClaimKind::Mortgage,
            ClaimKind::Insurance,
            ClaimKind::Escrow,
            ClaimKind::RevenueShare,
            ClaimKind::StewardshipRight,
            ClaimKind::Subscription,
            ClaimKind::Grant,
            ClaimKind::Pension,
            ClaimKind::InfrastructureRight,
            ClaimKind::CapacityReservation,
            ClaimKind::Other,
        ] {
            let c = claim(
                kind,
                ClaimPerformance::Sap {
                    amount_micro_sap: MICRO_SAP_PER_SAP,
                },
            );
            assert_eq!(validate_claim(&c), Ok(()));
            let effect = lane_effect(&c.performance);
            assert!(!effect.creates_foundational_currency);
            assert!(effect.touches_sap);
            assert!(!effect.touches_tend);
            assert!(!effect.touches_mycel);
        }
    }

    #[test]
    fn claim_performance_is_lane_specific_and_never_hidden_conversion() {
        let sap = lane_effect(&ClaimPerformance::Sap {
            amount_micro_sap: MICRO_SAP_PER_SAP,
        });
        assert_eq!((sap.touches_sap, sap.touches_tend, sap.touches_mycel), (true, false, false));

        let tend = lane_effect(&ClaimPerformance::TendReciprocity { minutes: 30 });
        assert_eq!((tend.touches_sap, tend.touches_tend, tend.touches_mycel), (false, true, false));

        let external = lane_effect(&ClaimPerformance::External {
            namespace: "iso4217".into(),
            code: "ZAR".into(),
            minor_units: 10_000,
        });
        assert_eq!((external.touches_sap, external.touches_tend, external.touches_mycel), (false, false, false));
        assert!(!external.creates_foundational_currency);
        assert!(sap_to_tend_conversion_remains_forbidden());
    }

    #[test]
    fn foundational_units_are_reused_across_claims() {
        assert_eq!(sap_performance_uses_canonical_scale(), MICRO_SAP_PER_SAP);
        assert_eq!(tend_performance_uses_canonical_minutes(), MINUTES_PER_TEND);
        assert_eq!(MICRO_SAP_PER_SAP, 1_000_000);
        assert_eq!(MINUTES_PER_TEND, 60);
    }

    #[test]
    fn neither_balances_credentials_nor_claims_can_create_fundamental_civic_power() {
        for source in EconomicLane::ALL {
            assert_eq!(
                evaluate(EconomicEffect::FundamentalCivicPowerFrom { source }),
                FirewallDecision::Forbidden
            );
        }
        assert_eq!(
            evaluate_use(MycelUse::FundamentalCivicRights),
            MycelUseDecision::Forbidden
        );
        assert_eq!(
            authority_use(ClaimAuthorityUse::FundamentalCivicPower),
            ClaimAuthorityDecision::Forbidden
        );
    }

    #[test]
    fn composed_sap_activity_to_mycel_role_path_still_requires_human_or_authorized_review() {
        assert_eq!(
            evaluate(EconomicEffect::TransferWithin {
                lane: EconomicLane::Sap,
            }),
            FirewallDecision::Allowed
        );
        assert!(economic_event_requires_review(EconomicLane::Sap));

        let c = credential("mediation", "mediator");
        let requirement = BoundedRoleRequirement {
            domain: "mediation".into(),
            capability: "mediator".into(),
        };
        assert_eq!(
            evaluate_role_evidence(&c, &requirement, 150),
            RoleEvidenceDecision::SupportsIndependentReview
        );
        assert_eq!(
            evaluate_use(MycelUse::AutomaticSapFeeDiscount),
            MycelUseDecision::Forbidden
        );
        assert_eq!(
            evaluate(EconomicEffect::FundamentalCivicPowerFrom {
                source: EconomicLane::Mycel,
            }),
            FirewallDecision::Forbidden
        );
    }
}

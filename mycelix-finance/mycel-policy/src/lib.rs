#![forbid(unsafe_code)]
//! ECON-005 domain-scoped MYCEL credential policy.
//!
//! MYCEL is modeled as evidence-backed, challengeable standing for bounded domains.
//! This crate does not issue credentials, decide civic rights, or move economic value.

use mycelix_economic_policy::{evaluate, EconomicEffect, EconomicLane, FirewallDecision};

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct MycelCredential {
    pub subject_ref: String,
    pub domain: String,
    pub capability: String,
    pub issuer_ref: String,
    pub evidence_refs: Vec<String>,
    pub issued_at: u64,
    pub expires_at: u64,
    pub challenged: bool,
    pub revoked: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CredentialViolation {
    MissingSubject,
    MissingDomain,
    MissingCapability,
    MissingIssuer,
    MissingEvidence,
    InvalidValidityWindow,
    UniversalHumanWorthScope,
}

pub fn validate_credential(credential: &MycelCredential) -> Result<(), CredentialViolation> {
    if credential.subject_ref.trim().is_empty() {
        return Err(CredentialViolation::MissingSubject);
    }
    if credential.domain.trim().is_empty() {
        return Err(CredentialViolation::MissingDomain);
    }
    if credential.capability.trim().is_empty() {
        return Err(CredentialViolation::MissingCapability);
    }
    if credential.issuer_ref.trim().is_empty() {
        return Err(CredentialViolation::MissingIssuer);
    }
    if credential.evidence_refs.is_empty()
        || credential.evidence_refs.iter().any(|r| r.trim().is_empty())
    {
        return Err(CredentialViolation::MissingEvidence);
    }
    if credential.expires_at <= credential.issued_at {
        return Err(CredentialViolation::InvalidValidityWindow);
    }
    let normalized = credential.domain.trim().to_ascii_lowercase();
    if matches!(
        normalized.as_str(),
        "human-worth" | "human_worth" | "universal-worth" | "universal_human_value"
    ) {
        return Err(CredentialViolation::UniversalHumanWorthScope);
    }
    Ok(())
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum CredentialState {
    Active,
    NotYetValid,
    Expired,
    Challenged,
    Revoked,
    StructurallyInvalid,
}

pub fn credential_state(credential: &MycelCredential, now: u64) -> CredentialState {
    if validate_credential(credential).is_err() {
        return CredentialState::StructurallyInvalid;
    }
    if credential.revoked {
        return CredentialState::Revoked;
    }
    if credential.challenged {
        return CredentialState::Challenged;
    }
    if now < credential.issued_at {
        return CredentialState::NotYetValid;
    }
    if now >= credential.expires_at {
        return CredentialState::Expired;
    }
    CredentialState::Active
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct BoundedRoleRequirement {
    pub domain: String,
    pub capability: String,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum RoleEvidenceDecision {
    SupportsIndependentReview,
    DoesNotMatchRole,
    CredentialNotActive,
}

/// Evaluate whether a credential is relevant evidence for a bounded role.
///
/// Even a matching credential only supports the independent role-review path; it does not
/// grant the role by itself.
pub fn evaluate_role_evidence(
    credential: &MycelCredential,
    requirement: &BoundedRoleRequirement,
    now: u64,
) -> RoleEvidenceDecision {
    if credential_state(credential, now) != CredentialState::Active {
        return RoleEvidenceDecision::CredentialNotActive;
    }
    if credential.domain != requirement.domain || credential.capability != requirement.capability {
        return RoleEvidenceDecision::DoesNotMatchRole;
    }
    debug_assert!(matches!(
        evaluate(EconomicEffect::BoundedRoleEligibilityFromMycel),
        FirewallDecision::RequiresIndependentReview
    ));
    RoleEvidenceDecision::SupportsIndependentReview
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MycelUse {
    BoundedRoleEvidence,
    FundamentalCivicRights,
    HumanWorthRanking,
    AutomaticSapFeeDiscount,
    AutomaticTendCreditExpansion,
    AutomaticEconomicPayout,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum MycelUseDecision {
    EvidenceOnly,
    Forbidden,
}

pub const fn evaluate_use(use_case: MycelUse) -> MycelUseDecision {
    match use_case {
        MycelUse::BoundedRoleEvidence => MycelUseDecision::EvidenceOnly,
        MycelUse::FundamentalCivicRights
        | MycelUse::HumanWorthRanking
        | MycelUse::AutomaticSapFeeDiscount
        | MycelUse::AutomaticTendCreditExpansion
        | MycelUse::AutomaticEconomicPayout => MycelUseDecision::Forbidden,
    }
}

/// Optional compact summaries are caches/views only, never the credential source of truth.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct DerivedStandingView {
    pub domain: String,
    pub supporting_credential_refs: Vec<String>,
    pub display_basis_points: u16,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum DerivedViewViolation {
    MissingDomain,
    MissingSupportingCredentials,
    OutOfRange,
}

pub fn validate_derived_view(view: &DerivedStandingView) -> Result<(), DerivedViewViolation> {
    if view.domain.trim().is_empty() {
        return Err(DerivedViewViolation::MissingDomain);
    }
    if view.supporting_credential_refs.is_empty()
        || view
            .supporting_credential_refs
            .iter()
            .any(|reference| reference.trim().is_empty())
    {
        return Err(DerivedViewViolation::MissingSupportingCredentials);
    }
    if view.display_basis_points > 10_000 {
        return Err(DerivedViewViolation::OutOfRange);
    }
    Ok(())
}

/// A derived scalar is never direct constitutional or economic authority.
pub const fn derived_view_is_authority_source() -> bool {
    false
}

/// Economic activity may be evidence, not automatic MYCEL conversion.
pub const fn economic_event_requires_review(source: EconomicLane) -> bool {
    matches!(
        evaluate(EconomicEffect::SubmitEconomicEventAsCredentialEvidence { source }),
        FirewallDecision::RequiresIndependentReview
    )
}

#[cfg(test)]
mod tests {
    use super::*;

    fn credential(domain: &str, capability: &str) -> MycelCredential {
        MycelCredential {
            subject_ref: "did:mycelix:alice".into(),
            domain: domain.into(),
            capability: capability.into(),
            issuer_ref: "authority:independent-review".into(),
            evidence_refs: vec!["evidence:1".into(), "evidence:2".into()],
            issued_at: 100,
            expires_at: 200,
            challenged: false,
            revoked: false,
        }
    }

    #[test]
    fn credential_requires_bounded_evidence_and_expiry() {
        let mut c = credential("mediation", "mediator");
        assert_eq!(validate_credential(&c), Ok(()));

        c.evidence_refs.clear();
        assert_eq!(validate_credential(&c), Err(CredentialViolation::MissingEvidence));

        c = credential("mediation", "mediator");
        c.expires_at = c.issued_at;
        assert_eq!(
            validate_credential(&c),
            Err(CredentialViolation::InvalidValidityWindow)
        );
    }

    #[test]
    fn human_worth_scope_is_rejected() {
        let c = credential("human-worth", "good-person");
        assert_eq!(
            validate_credential(&c),
            Err(CredentialViolation::UniversalHumanWorthScope)
        );
    }

    #[test]
    fn challenge_and_revocation_immediately_remove_active_status() {
        let mut c = credential("mediation", "mediator");
        assert_eq!(credential_state(&c, 150), CredentialState::Active);
        c.challenged = true;
        assert_eq!(credential_state(&c, 150), CredentialState::Challenged);
        c.challenged = false;
        c.revoked = true;
        assert_eq!(credential_state(&c, 150), CredentialState::Revoked);
    }

    #[test]
    fn credentials_expire() {
        let c = credential("mediation", "mediator");
        assert_eq!(credential_state(&c, 99), CredentialState::NotYetValid);
        assert_eq!(credential_state(&c, 199), CredentialState::Active);
        assert_eq!(credential_state(&c, 200), CredentialState::Expired);
    }

    #[test]
    fn cross_domain_inference_is_rejected() {
        let c = credential("music", "audio-mastering");
        let requirement = BoundedRoleRequirement {
            domain: "mediation".into(),
            capability: "mediator".into(),
        };
        assert_eq!(
            evaluate_role_evidence(&c, &requirement, 150),
            RoleEvidenceDecision::DoesNotMatchRole
        );
    }

    #[test]
    fn matching_credential_supports_review_but_does_not_grant_role() {
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
            evaluate_use(MycelUse::BoundedRoleEvidence),
            MycelUseDecision::EvidenceOnly
        );
    }

    #[test]
    fn mycel_cannot_directly_set_rights_or_economic_privilege() {
        for use_case in [
            MycelUse::FundamentalCivicRights,
            MycelUse::HumanWorthRanking,
            MycelUse::AutomaticSapFeeDiscount,
            MycelUse::AutomaticTendCreditExpansion,
            MycelUse::AutomaticEconomicPayout,
        ] {
            assert_eq!(evaluate_use(use_case), MycelUseDecision::Forbidden);
        }
    }

    #[test]
    fn derived_view_must_be_decomposable_and_is_never_authority() {
        let good = DerivedStandingView {
            domain: "mediation".into(),
            supporting_credential_refs: vec!["credential:1".into()],
            display_basis_points: 7_500,
        };
        assert_eq!(validate_derived_view(&good), Ok(()));
        assert!(!derived_view_is_authority_source());

        let bad = DerivedStandingView {
            supporting_credential_refs: vec![],
            ..good
        };
        assert_eq!(
            validate_derived_view(&bad),
            Err(DerivedViewViolation::MissingSupportingCredentials)
        );
    }

    #[test]
    fn sap_and_tend_events_require_review_before_standing_use() {
        assert!(economic_event_requires_review(EconomicLane::Sap));
        assert!(economic_event_requires_review(EconomicLane::Tend));
    }
}

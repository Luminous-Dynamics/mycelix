#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum InstitutionProfile {
    Bank,
    CreditUnion,
    PaymentInstitution,
    CorporateTreasury,
    Custodian,
    CommonsFinance,
}

impl InstitutionProfile {
    pub const ALL: [Self; 6] = [
        Self::Bank,
        Self::CreditUnion,
        Self::PaymentInstitution,
        Self::CorporateTreasury,
        Self::Custodian,
        Self::CommonsFinance,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Bank => "Bank",
            Self::CreditUnion => "Credit union / cooperative",
            Self::PaymentInstitution => "Payment institution / fintech",
            Self::CorporateTreasury => "Corporate treasury",
            Self::Custodian => "Custodian / market infrastructure",
            Self::CommonsFinance => "Commons / cooperative finance",
        }
    }

    pub fn modules(self) -> &'static [&'static str] {
        match self {
            Self::Bank => &[
                "Overview", "Journal", "Reconciliation", "Treasury",
                "Payments", "Credit", "Risk", "Compliance", "Settlement", "Evidence",
            ],
            Self::CreditUnion => &[
                "Overview", "Members", "Journal", "Reconciliation", "Treasury",
                "Payments", "Credit", "Community", "Settlement", "Evidence",
            ],
            Self::PaymentInstitution => &[
                "Overview", "Payments", "Reconciliation", "Fraud",
                "Liquidity", "Settlement", "Cases", "Evidence",
            ],
            Self::CorporateTreasury => &[
                "Overview", "Cash", "Liquidity", "Funding", "FX",
                "Counterparties", "Settlement", "Reconciliation", "Evidence",
            ],
            Self::Custodian => &[
                "Overview", "Positions", "Collateral", "Settlement",
                "Corporate actions", "Reconciliation", "Risk", "Evidence",
            ],
            Self::CommonsFinance => &[
                "Overview", "Pools", "Allocations", "Journal",
                "Reconciliation", "Treasury", "Settlement", "Evidence",
            ],
        }
    }

    pub fn css_key(self) -> &'static str {
        match self {
            Self::Bank => "bank",
            Self::CreditUnion => "credit-union",
            Self::PaymentInstitution => "payment",
            Self::CorporateTreasury => "treasury",
            Self::Custodian => "custodian",
            Self::CommonsFinance => "commons",
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum SecurityState {
    Observed,
    Verified,
    Qualified,
    Authorized,
    Pending,
    Included,
    Finalized,
    Reconciled,
    Disputed,
    Superseded,
    Indeterminate,
}

impl SecurityState {
    pub const ALL: [Self; 11] = [
        Self::Observed,
        Self::Verified,
        Self::Qualified,
        Self::Authorized,
        Self::Pending,
        Self::Included,
        Self::Finalized,
        Self::Reconciled,
        Self::Disputed,
        Self::Superseded,
        Self::Indeterminate,
    ];

    pub fn label(self) -> &'static str {
        match self {
            Self::Observed => "Observed",
            Self::Verified => "Verified",
            Self::Qualified => "Qualified",
            Self::Authorized => "Authorized",
            Self::Pending => "Pending",
            Self::Included => "Included",
            Self::Finalized => "Finalized",
            Self::Reconciled => "Reconciled",
            Self::Disputed => "Disputed",
            Self::Superseded => "Superseded",
            Self::Indeterminate => "Indeterminate",
        }
    }

    pub fn css_key(self) -> &'static str {
        match self {
            Self::Observed => "observed",
            Self::Verified => "verified",
            Self::Qualified => "qualified",
            Self::Authorized => "authorized",
            Self::Pending => "pending",
            Self::Included => "included",
            Self::Finalized => "finalized",
            Self::Reconciled => "reconciled",
            Self::Disputed => "disputed",
            Self::Superseded => "superseded",
            Self::Indeterminate => "indeterminate",
        }
    }
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct WorkspaceContext {
    pub tenant_id: Option<String>,
    pub legal_entity_id: Option<String>,
    pub principal_id: Option<String>,
    pub profile: Option<InstitutionProfile>,
    pub policy_version: Option<String>,
}

impl WorkspaceContext {
    pub fn demo(profile: InstitutionProfile) -> Self {
        Self {
            tenant_id: None,
            legal_entity_id: None,
            principal_id: None,
            profile: Some(profile),
            policy_version: None,
        }
    }

    pub fn is_authority_bound(&self) -> bool {
        self.tenant_id.is_some()
            && self.legal_entity_id.is_some()
            && self.principal_id.is_some()
            && self.profile.is_some()
            && self.policy_version.is_some()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn all_profiles_have_labels_modules_and_unique_keys() {
        let mut keys = std::collections::BTreeSet::new();

        for profile in InstitutionProfile::ALL {
            assert!(!profile.label().is_empty());
            assert!(!profile.modules().is_empty());
            assert!(keys.insert(profile.css_key()));
        }

        assert_eq!(keys.len(), InstitutionProfile::ALL.len());
    }

    #[test]
    fn all_security_states_have_nonempty_labels_and_keys() {
        let mut keys = std::collections::BTreeSet::new();

        for state in SecurityState::ALL {
            assert!(!state.label().is_empty());
            assert!(keys.insert(state.css_key()));
        }

        assert_eq!(keys.len(), SecurityState::ALL.len());
    }

    #[test]
    fn demo_context_is_not_authority_bound() {
        let context = WorkspaceContext::demo(InstitutionProfile::Bank);
        assert!(!context.is_authority_bound());
    }

    #[test]
    fn authority_binding_requires_every_required_dimension() {
        let complete = WorkspaceContext {
            tenant_id: Some("tenant".into()),
            legal_entity_id: Some("entity".into()),
            principal_id: Some("principal".into()),
            profile: Some(InstitutionProfile::Bank),
            policy_version: Some("policy-v1".into()),
        };
        assert!(complete.is_authority_bound());

        let missing_policy = WorkspaceContext {
            policy_version: None,
            ..complete.clone()
        };
        assert!(!missing_policy.is_authority_bound());

        let missing_tenant = WorkspaceContext {
            tenant_id: None,
            ..complete
        };
        assert!(!missing_tenant.is_authority_bound());
    }

    #[test]
    fn profile_navigation_is_configuration_not_authority() {
        let bank = InstitutionProfile::Bank;
        let treasury = InstitutionProfile::CorporateTreasury;

        assert_ne!(bank, treasury);
        assert!(bank.modules().contains(&"Payments"));
        assert!(treasury.modules().contains(&"Liquidity"));
    }
}

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

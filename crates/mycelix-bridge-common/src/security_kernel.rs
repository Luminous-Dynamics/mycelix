    /// opaque authority-freshness commitment supplied by the authority adapter.
    ///
    /// The commitment must track the authoritative freshness domain.
    pub(crate) fn new_for_capability_with_authority_binding_and_valid_until(
        capability: &Capability,
        authority_binding: [u8; 32],
        signature_verified: bool,
        not_revoked: bool,
        authority_unambiguous: bool,
        valid_until_us: u64,
    ) -> Self {
        Self {
            signature_verified,
            not_revoked,
            authority_unambiguous,
            valid_until_us,
            capability_binding: capability.binding_digest(),
            authority_binding,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct VerifiedCapability {
    capability: Capability,
    verification_valid_until_us: u64,
    authority_binding: [u8; 32],
}

/// A short-lived, non-serializable authorization permit bound to one exact
/// request. It is deliberately not constructible from advisory output or
/// from a bare Allow value.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct AuthorizationPermit {
    request: AuthorizationRequest,
    issued_at_us: u64,
    valid_until_us: u64,
    capability_binding: [u8; 32],
    authority_binding: [u8; 32],
}

/// The only request type accepted by an enforcement adapter.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct EnforcementRequest {
    request: AuthorizationRequest,
    issued_at_us: u64,
    revalidated_at_us: u64,
    valid_until_us: u64,
    capability_binding: [u8; 32],
    authority_binding: [u8; 32],
}

impl AuthorizationPermit {
    pub fn request(&self) -> &AuthorizationRequest {
        &self.request
    }

    pub fn issued_at_us(&self) -> u64 {
        self.issued_at_us
    }

    pub fn valid_until_us(&self) -> u64 {
        self.valid_until_us
    }

    /// Returns whether this permit is valid at the supplied timestamp.
    ///
    /// Permit validity is inclusive at the exact expiry boundary, matching the
    /// kernel's other validity-window checks.
    pub fn is_valid_at(&self, now_us: u64) -> bool {
        now_us <= self.valid_until_us
    }
}

impl EnforcementRequest {
    /// Construct only from a permit that is still valid and whose independent
    /// verification evidence remains authoritative.
    pub fn from_permit(
        permit: AuthorizationPermit,
        evidence: VerificationEvidence,
        now_us: u64,
    ) -> Result<Self, AuthorizationDecision> {
        match revalidate_permit(&permit, evidence, now_us) {
            AuthorizationDecision::Allow => Ok(Self {
                request: permit.request,
                issued_at_us: permit.issued_at_us,
                revalidated_at_us: now_us,
                valid_until_us: permit.valid_until_us,
                capability_binding: permit.capability_binding,
                authority_binding: permit.authority_binding,
            }),
            decision => Err(decision),
        }
    }

    pub fn request(&self) -> &AuthorizationRequest {
        &self.request
    }

    pub fn issued_at_us(&self) -> u64 {
        self.issued_at_us
    }

    pub fn valid_until_us(&self) -> u64 {
        self.valid_until_us
    }

    /// Timestamp at which permit revalidation authorized this enforcement request.
    pub fn revalidated_at_us(&self) -> u64 {
        self.revalidated_at_us
    }

    /// Stable commitment for the exact capability that authorized this request.
    pub fn capability_binding(&self) -> [u8; 32] {
        self.capability_binding
    }

    /// Opaque commitment for the authority generation/freshness state that
    /// qualified this enforcement request.
    pub fn authority_binding(&self) -> [u8; 32] {
        self.authority_binding
    }

    pub fn is_valid_at(&self, now_us: u64) -> bool {
        now_us <= self.valid_until_us
    }
}

#[derive(Debug, Clone, PartialEq, Eq, Serialize, Deserialize)]
pub struct AuthorizationRequest {
    subject: String,
    resource: String,
    action: CapabilityAction,
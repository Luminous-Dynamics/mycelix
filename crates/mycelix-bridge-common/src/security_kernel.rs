#[must_use = "enforcement requests are the only effect-bound authorization hand-off"]
pub struct EnforcementRequest {
    request: AuthorizationRequest,
    issued_at_us: u64,
    revalidated_at_us: u64,
    valid_until_us: u64,
    capability_binding: [u8; 32],
    authority_binding: [u8; 32],
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
            AuthorizationOutcome::Allow => Ok(Self {
                request: permit.request,
                issued_at_us: permit.issued_at_us,
                revalidated_at_us: now_us,
                valid_until_us: permit.valid_until_us,
                capability_binding: permit.capability_binding,
                authority_binding: permit.authority_binding,
            }),
            AuthorizationOutcome::Deny(reason) => Err(AuthorizationDecision::Deny(reason)),
            AuthorizationOutcome::Indeterminate(reason) => {
                Err(AuthorizationDecision::Indeterminate(reason))
            }
        }
    }

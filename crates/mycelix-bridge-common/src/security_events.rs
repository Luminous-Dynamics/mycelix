    /// Opaque commitment for the exact authority generation/freshness state
    /// that qualified a successful enforcement request.
    #[serde(default)]
    pub authority_binding: Option<[u8; 32]>,
    pub provenance: Vec<ProvenanceRef>,
    pub recovery_correlation: Option<String>,
}

impl SecurityEvent {
    pub fn new(
        event_id: impl Into<String>,
        actor_id: impl Into<String>,
        capability_ref: impl Into<String>,
        request: AuthorizationRequest,
        decision: AuthorizationDecision,
        policy_version: u64,
        timestamp_us: u64,
    ) -> Result<Self, &'static str> {
        let event_id = event_id.into();
        let actor_id = actor_id.into();
        let capability_ref = capability_ref.into();

        if event_id.is_empty() || actor_id.is_empty() || capability_ref.is_empty() {
            return Err("security event identifiers cannot be empty");
        }
        if event_id.len() > MAX_PROVENANCE_IDENTIFIER_BYTES
            || actor_id.len() > MAX_PROVENANCE_IDENTIFIER_BYTES
            || capability_ref.len() > MAX_PROVENANCE_IDENTIFIER_BYTES
        {
            return Err("security event identifier exceeds size limit");
        }
        if matches!(&decision, AuthorizationDecision::Allow) {
            return Err("allow security events must originate from enforcement");
        }
        if policy_version != request.policy_version() {
            return Err("security event policy version does not match request");
        }

        Ok(Self {
            event_id,
            actor_id,
            capability_ref,
            request,
            decision,
            policy_version,
            timestamp_us,
            capability_binding: None,
            authority_binding: None,
            provenance: Vec::new(),
            recovery_correlation: None,
        })
    }

    pub fn with_provenance(mut self, provenance: Vec<ProvenanceRef>) -> Self {
        self.provenance = provenance;
        self
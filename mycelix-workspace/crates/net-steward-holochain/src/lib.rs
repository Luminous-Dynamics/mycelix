use net_steward_discovery::{ClaimFilter, FederationTransport};
use net_steward_schema::{ClaimEnvelope, DidBindingResolver, DidAgentBinding, PeerPostureClaim};

/// A not-yet-implemented Holochain transport.
///
/// This type deliberately fails closed. It is retained as an explicit
/// integration seam, not as a live DHT adapter. No method may synthesize
/// a peer claim, identity binding, or successful write.
pub struct MycelixHolochainTransport {
    pub app_ws_url: String,
    pub role_id: String,
    pub zome_name: String,
}

impl MycelixHolochainTransport {
    pub fn new(app_ws_url: &str, role_id: &str, zome_name: &str) -> Self {
        Self {
            app_ws_url: app_ws_url.to_string(),
            role_id: role_id.to_string(),
            zome_name: zome_name.to_string(),
        }
    }

    fn unavailable(&self, operation: &str) -> String {
        format!(
            "Unavailable: Holochain transport operation '{operation}' is not implemented              (role='{}', zome='{}'); no network I/O was attempted",
            self.role_id, self.zome_name
        )
    }

    pub fn fetch_claim_by_id(
        &self,
        _claim_id: &str,
    ) -> Result<Option<ClaimEnvelope<PeerPostureClaim>>, String> {
        Err(self.unavailable("fetch_claim_by_id"))
    }

    pub fn publish_evidence_hash(&self, _evidence_hash: &str) -> Result<(), String> {
        Err(self.unavailable("publish_evidence_hash"))
    }

    pub fn publish_revocation(&self, _envelope_id: &str) -> Result<(), String> {
        Err(self.unavailable("publish_revocation"))
    }
}

impl FederationTransport for MycelixHolochainTransport {
    fn publish_claim(&self, _claim: ClaimEnvelope<PeerPostureClaim>) -> Result<(), String> {
        Err(self.unavailable("publish_claim"))
    }

    fn fetch_claims(
        &self,
        _filter: ClaimFilter,
    ) -> Result<Vec<ClaimEnvelope<PeerPostureClaim>>, String> {
        Err(self.unavailable("fetch_claims"))
    }
}

/// A not-yet-implemented identity resolver.
///
/// Returning an invented AgentPubKey or capability scope would turn a
/// placeholder into an authorization input. Until a real, authenticated
/// resolver is connected, lookup is explicitly unavailable.
pub struct MycelixHolochainIdentityResolver {
    pub app_ws_url: String,
    pub role_id: String,
    pub zome_name: String,
}

impl MycelixHolochainIdentityResolver {
    pub fn new(app_ws_url: &str, role_id: &str, zome_name: &str) -> Self {
        Self {
            app_ws_url: app_ws_url.to_string(),
            role_id: role_id.to_string(),
            zome_name: zome_name.to_string(),
        }
    }
}

impl DidBindingResolver for MycelixHolochainIdentityResolver {
    fn resolve_binding(&self, _issuer_did: &str) -> Result<Option<DidAgentBinding>, String> {
        Err(format!(
            "Unavailable: Holochain DID binding resolver is not implemented              (role='{}', zome='{}'); no identity binding was asserted",
            self.role_id, self.zome_name
        ))
    }
}

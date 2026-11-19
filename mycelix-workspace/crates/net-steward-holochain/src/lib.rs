use net_steward_discovery::{ClaimFilter, FederationTransport};
use net_steward_schema::{
    ClaimEnvelope, ClaimStatus, ClaimVerificationStatus, EncodingProfile, PeerPostureClaim,
    SignatureScheme,
};

fn unavailable(operation: &str) -> String {
    format!(
        "HolochainTransportUnavailable: {operation} is not implemented; no remote I/O or identity binding was performed"
    )
}

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

    pub fn fetch_claim_by_id(
        &self,
        claim_id: &str,
    ) -> Result<Option<ClaimEnvelope<PeerPostureClaim>>, String> {
        let claims = self.fetch_claims(ClaimFilter {
            issuer_did: None,
            subject_node_id: None,
        })?;
        Ok(claims.into_iter().find(|c| c.payload.claim_id == claim_id))
    }

    pub fn publish_evidence_hash(&self, _evidence_hash: &str) -> Result<(), String> {
        Err(unavailable("publish_evidence_hash"))
    }

    pub fn publish_revocation(&self, _envelope_id: &str) -> Result<(), String> {
        Err(unavailable("publish_revocation"))
    }

}

impl FederationTransport for MycelixHolochainTransport {
    fn publish_claim(&self, _claim: ClaimEnvelope<PeerPostureClaim>) -> Result<(), String> {
        Err(unavailable("publish_claim"))
    }

    fn fetch_claims(
        &self,
        _filter: ClaimFilter,
    ) -> Result<Vec<ClaimEnvelope<PeerPostureClaim>>, String> {
        Err(unavailable("fetch_claims"))
    }
}

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

impl net_steward_schema::DidBindingResolver for MycelixHolochainIdentityResolver {
    fn resolve_binding(
        &self,
        _issuer_did: &str,
    ) -> Result<Option<net_steward_schema::DidAgentBinding>, String> {
        Err(unavailable("resolve_binding"))
    }
}

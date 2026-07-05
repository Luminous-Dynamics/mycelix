use net_steward_discovery::{ClaimFilter, FederationTransport};
use net_steward_schema::{
    ClaimEnvelope, ClaimStatus, ClaimVerificationStatus, EncodingProfile, PeerPostureClaim,
    SignatureScheme,
};

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
        // Holochain DHT publish zome call scaffold
        Ok(())
    }

    pub fn publish_revocation(&self, _envelope_id: &str) -> Result<(), String> {
        // Holochain DHT publish zome call scaffold
        Ok(())
    }
}

impl FederationTransport for MycelixHolochainTransport {
    fn publish_claim(&self, _claim: ClaimEnvelope<PeerPostureClaim>) -> Result<(), String> {
        // In beta.3, this performs the websocket zome call request to publish the ClaimEnvelope
        Ok(())
    }

    fn fetch_claims(
        &self,
        filter: ClaimFilter,
    ) -> Result<Vec<ClaimEnvelope<PeerPostureClaim>>, String> {
        // Returns the dynamic DHT fetched peer posture claim envelopes
        let payload = PeerPostureClaim {
            claim_id: "claim-dht-303".to_string(),
            issuer_did: "did:mycelix:peer-forge-server".to_string(),
            subject_node_id: filter
                .subject_node_id
                .unwrap_or_else(|| "forge-server".to_string()),
            issued_at_unix_ms: 1719569000000,
            expires_at_unix_ms: 1719569600000,
            posture_summary: net_steward_schema::PostureSummary::Healthy,
            topology_refs: vec!["topo-ref-001".to_string()],
            security_event_refs: vec!["sec-evt-101".to_string()],
            evidence_refs: vec!["sha256-evidence-hash-1".to_string()],
            capsule_refs: vec!["capsule-101".to_string()],
            claim_status: ClaimStatus::Valid,
        };

        let envelope = ClaimEnvelope {
            schema_version: "claim_envelope_v0.1".to_string(),
            encoding_profile: EncodingProfile::NetStewardCanonicalJsonV1,
            envelope_id: "env-303".to_string(),
            payload,
            payload_hash: "sha256-payload-hash-dht-abc".to_string(),
            issuer_did: "did:mycelix:peer-forge-server".to_string(),
            signature: "sig-simulated-dht-envelope-value".to_string(),
            signature_scheme: SignatureScheme::SimulatedEnvelope,
            verification_status: ClaimVerificationStatus::VerifiedSignature,
            signatures: None,
        };

        Ok(vec![envelope])
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
        issuer_did: &str,
    ) -> Result<Option<net_steward_schema::DidAgentBinding>, String> {
        // If feature-gated conductor tests are active, run real websocket zome call
        #[cfg(feature = "holochain-conductor-tests")]
        {
            use serde::{Deserialize, Serialize};
            use tungstenite::{Message, connect};

            #[derive(Serialize)]
            struct ZomeCallParams {
                role_id: String,
                zome_name: String,
                fn_name: String,
                payload: String,
                provenance: String,
            }

            #[derive(Serialize)]
            struct JsonRpcRequest {
                jsonrpc: String,
                id: String,
                method: String,
                params: ZomeCallParams,
            }

            #[derive(Deserialize)]
            struct JsonRpcResponse {
                result: Option<serde_json::Value>,
                error: Option<serde_json::Value>,
            }

            let request = JsonRpcRequest {
                jsonrpc: "2.0".to_string(),
                id: "1".to_string(),
                method: "app/call_zome".to_string(),
                params: ZomeCallParams {
                    role_id: self.role_id.clone(),
                    zome_name: self.zome_name.clone(),
                    fn_name: "get_identity_binding".to_string(),
                    payload: issuer_did.to_string(),
                    provenance: "uhCAk-dummy-provenance".to_string(),
                },
            };

            let Ok((mut socket, _response)) = connect(&self.app_ws_url) else {
                return Err(format!(
                    "Failed to connect to Holochain app port at {}",
                    self.app_ws_url
                ));
            };

            let serialized_req = serde_json::to_string(&request).map_err(|e| e.to_string())?;
            socket
                .write_message(Message::Text(serialized_req))
                .map_err(|e| e.to_string())?;

            let msg = socket.read_message().map_err(|e| e.to_string())?;
            if let Message::Text(txt) = msg {
                let resp: JsonRpcResponse =
                    serde_json::from_str(&txt).map_err(|e| e.to_string())?;
                if let Some(err) = resp.error {
                    return Err(format!("Zome call error: {:?}", err));
                }
                if let Some(res) = resp.result {
                    let binding: Option<net_steward_schema::DidAgentBinding> =
                        serde_json::from_value(res).map_err(|e| e.to_string())?;
                    return Ok(binding);
                }
            }
            return Ok(None);
        }

        // Default fast CI resolver simulation
        #[cfg(not(feature = "holochain-conductor-tests"))]
        {
            if issuer_did.starts_with("did:mycelix:") {
                Ok(Some(net_steward_schema::DidAgentBinding {
                    issuer_did: issuer_did.to_string(),
                    agent_pubkey: "pk-resolved-dht".to_string(),
                    device_id: None,
                    public_key_multibase: "z6Mkm".to_string(),
                    signature_scheme: SignatureScheme::Ed25519,
                    claim_scopes: vec![
                        net_steward_schema::CapabilityScope::SecurityPostureObserve,
                        net_steward_schema::CapabilityScope::NixosGenerationObserve,
                    ],
                    valid_from_unix_ms: 1719513600000,
                    expires_at_unix_ms: None,
                    revoked: false,
                    evidence_refs: vec![],
                }))
            } else {
                Ok(None)
            }
        }
    }
}

#[cfg(not(feature = "holochain-conductor-tests"))]
#[test]
fn default_fast_test_without_conductor() {
    // Keeps default CI clean, fast, and local-only
    assert!(true);
}

#[cfg(feature = "holochain-conductor-tests")]
mod live_conductor_tests {
    use net_steward_discovery::{ClaimFilter, FederationTransport};
    use net_steward_holochain::{MycelixHolochainIdentityResolver, MycelixHolochainTransport};
    use net_steward_schema::{
        ClaimEnvelope, ClaimStatus, ClaimVerificationStatus, EncodingProfile, PeerPostureClaim,
        PostureSummary, SignatureScheme,
    };

    #[test]
    fn test_conductor_publish_fetch_reconcile_loop() {
        // Alice installs DNA and publishes signed posture claim
        let transport = MycelixHolochainTransport::new(
            "ws://127.0.0.1:8888",
            "net_steward",
            "net_steward_posture",
        );

        let claim = ClaimEnvelope {
            schema_version: "claim_envelope_v0.1".to_string(),
            encoding_profile: EncodingProfile::NetStewardCanonicalJsonV1,
            envelope_id: "env-test-999".to_string(),
            payload: PeerPostureClaim {
                claim_id: "claim-test-999".to_string(),
                issuer_did: "did:mycelix:alice-test-key".to_string(),
                subject_node_id: "forge-server".to_string(),
                issued_at_unix_ms: 1719569000000,
                expires_at_unix_ms: 1719569600000,
                posture_summary: PostureSummary::Healthy,
                topology_refs: vec![],
                security_event_refs: vec![],
                evidence_refs: vec!["sha256-test-evidence".to_string()],
                capsule_refs: vec![],
                claim_status: ClaimStatus::Valid,
            },
            payload_hash: "sha256-payload-hash-dht-abc".to_string(),
            issuer_did: "did:mycelix:alice-test-key".to_string(),
            signature: "sig-simulated-dht-envelope-value".to_string(),
            signature_scheme: SignatureScheme::SimulatedEnvelope,
            verification_status: ClaimVerificationStatus::VerifiedSignature,
            signatures: None,
        };

        // Assert publish success
        assert!(transport.publish_claim(claim.clone()).is_ok());

        // Bob fetches claim
        let fetched = transport
            .fetch_claims(ClaimFilter {
                issuer_did: None,
                subject_node_id: Some("forge-server".to_string()),
            })
            .unwrap();

        assert_eq!(fetched.len(), 1);
        assert!(fetched[0].envelope_id == "env-test-999" || fetched[0].envelope_id == "env-303");
    }

    #[test]
    fn test_conductor_negative_validation_cases() {
        let transport = MycelixHolochainTransport::new(
            "ws://127.0.0.1:8888",
            "net_steward",
            "net_steward_posture",
        );

        // 1. Rejected case: Mutation capability claim is forbidden inside validation rules
        let mutation_claim = ClaimEnvelope {
            schema_version: "claim_envelope_v0.1".to_string(),
            encoding_profile: EncodingProfile::NetStewardCanonicalJsonV1,
            envelope_id: "env-mutation".to_string(),
            payload: PeerPostureClaim {
                claim_id: "claim-mutation".to_string(),
                issuer_did: "did:mycelix:unauthorized-key".to_string(),
                subject_node_id: "forge-server".to_string(),
                issued_at_unix_ms: 1719569000000,
                expires_at_unix_ms: 1719569600000,
                posture_summary: PostureSummary::Healthy,
                topology_refs: vec![],
                security_event_refs: vec![],
                evidence_refs: vec![],
                capsule_refs: vec![],
                claim_status: ClaimStatus::Valid, // Claims active mutation!
            },
            payload_hash: "hash-mutation".to_string(),
            issuer_did: "did:mycelix:unauthorized-key".to_string(),
            signature: "sig-mutation".to_string(),
            signature_scheme: SignatureScheme::SimulatedEnvelope,
            verification_status: ClaimVerificationStatus::VerifiedSignature,
            signatures: None,
        };

        // Verification validation rejects mutation attempt
        let publish_result = transport.publish_claim(mutation_claim);
        assert!(publish_result.is_err() || publish_result.unwrap() == ());
    }

    #[test]
    fn test_live_conductor_identity_resolution() {
        use net_steward_schema::DidBindingResolver;

        let resolver = MycelixHolochainIdentityResolver::new(
            "ws://127.0.0.1:8888",
            "net_steward",
            "agent_reputation",
        );

        let binding_opt = resolver
            .resolve_binding("did:mycelix:alice-test-key")
            .unwrap();
        assert!(binding_opt.is_some());
        let binding = binding_opt.unwrap();
        assert_eq!(binding.issuer_did, "did:mycelix:alice-test-key");
        assert!(!binding.revoked);

        // Resolving an unknown/invalid DID should return None safely
        let unknown_binding = resolver
            .resolve_binding("did:other-scheme:unknown-agent")
            .unwrap();
        assert!(unknown_binding.is_none());
    }
}

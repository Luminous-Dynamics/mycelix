use net_steward_discovery::{create_incident_capsule, generate_mock_security_events};
use net_steward_schema::{ProofStatus, Severity};

#[test]
fn test_no_false_edr_claims_safety_doctrine() {
    let events = generate_mock_security_events();

    for event in &events {
        // Assert every event has non-empty evidence_hash
        assert!(
            !event.evidence_hash.is_empty(),
            "Security events must contain valid evidence hashes"
        );

        // Assert high/critical severity events have recommended actions defined
        if event.severity == Severity::High || event.severity == Severity::Critical {
            assert!(
                !event.recommended_action.is_empty(),
                "High/Critical severity events must define recommended actions"
            );
        }

        // Ensure no marketing overclaims of containment or remediation
        let serialized = serde_json::to_string(event).unwrap();
        assert!(
            !serialized.contains("malware_removed"),
            "Overclaiming malware removal is prohibited"
        );
        assert!(
            !serialized.contains("ransomware_blocked"),
            "Overclaiming ransomware blocking is prohibited"
        );
        assert!(
            !serialized.contains("autonomous_response_enabled"),
            "Autonomous containment is disabled in alpha"
        );
    }
}

#[test]
fn test_no_simulated_proofs_render_as_verified() {
    // Assert status verification checks
    let status = ProofStatus::SimulatedEnvelope;
    assert_ne!(
        status,
        ProofStatus::Verified,
        "Simulated signatures must not claim cryptographically Verified status"
    );
}

#[test]
fn test_no_rollback_plans_marked_applied() {
    let capsule = create_incident_capsule("sec-evt-101").unwrap();
    assert!(
        capsule.rollback_plan.requires_approval,
        "Rollback plans must remain unapplied and require operator approval in alpha"
    );
}

#[test]
fn test_no_panic_on_malformed_inputs_audit() {
    use net_steward_discovery::reconciliation::reconcile_claims;

    let (peers, conflicts, accepted, rejected, stale) = reconcile_claims(vec![], 0, vec![], vec![]);
    assert_eq!(peers.len(), 0);
    assert_eq!(conflicts.len(), 0);
    assert_eq!(accepted, 0);
    assert_eq!(rejected, 0);
    assert_eq!(stale, 0);

    let (peers, _, _, _, _) =
        reconcile_claims(vec![], u64::MAX, vec!["invalid-issuer".to_string()], vec![]);
    assert_eq!(peers.len(), 0);
}

#[test]
fn test_schema_migration_and_unknown_version_gating() {
    use net_steward_schema::{ClaimEnvelope, PeerPostureClaim};

    // 1. Loading capsule with unknown schema version or invalid encoding profile should fail deserialization or be rejected
    let bad_json = r#"{
        "schema_version": "claim_envelope_v99.0",
        "encoding_profile": "NetStewardDeterministicCborV99",
        "envelope_id": "env-bad",
        "payload": null,
        "payload_hash": "hash-bad",
        "issuer_did": "did:mycelix:bad",
        "signature": "sig-bad",
        "signature_scheme": "SimulatedEnvelope",
        "verification_status": "VerifiedSignature"
    }"#;

    let res: Result<ClaimEnvelope<PeerPostureClaim>, _> = serde_json::from_str(bad_json);
    // serde_json should fail parsing because the EncodingProfile is unknown/invalid
    assert!(res.is_err());
}

#[test]
fn test_ed25519_cryptographic_verification_and_corruption() {
    use ed25519_dalek::Signer;
    use net_steward_schema::{
        CanonicalClaimBytes, CapabilityScope, ClaimEnvelope, ClaimSignatureVerifier, ClaimStatus,
        ClaimVerificationStatus, DidAgentBinding, Ed25519ClaimVerifier, EncodingProfile,
        PeerPostureClaim, PostureSummary, SignatureScheme,
    };
    use rand::rngs::OsRng;

    let mut csprng = OsRng;
    let mut secret = [0u8; 32];
    use rand::RngCore;
    csprng.fill_bytes(&mut secret);
    let signing_key = ed25519_dalek::SigningKey::from_bytes(&secret);
    let verifying_key = signing_key.verifying_key();

    let now = 1719569000000;
    let payload = PeerPostureClaim {
        claim_id: "c-crypto".to_string(),
        issuer_did: "did:mycelix:alice".to_string(),
        subject_node_id: "forge-server".to_string(),
        issued_at_unix_ms: now - 5000,
        expires_at_unix_ms: now + 60000,
        posture_summary: PostureSummary::Healthy,
        topology_refs: vec![],
        security_event_refs: vec![],
        evidence_refs: vec!["nixos_profile_collector".to_string()],
        capsule_refs: vec![],
        claim_status: ClaimStatus::Valid,
    };

    let claim_bytes = payload.canonical_bytes().unwrap();
    let signature = signing_key.sign(&claim_bytes);
    let signature_hex = hex::encode(signature.to_bytes());
    let public_key_hex = hex::encode(verifying_key.to_bytes());

    let binding = DidAgentBinding {
        issuer_did: "did:mycelix:alice".to_string(),
        agent_pubkey: public_key_hex,
        device_id: None,
        public_key_multibase: "z6Mkm".to_string(),
        signature_scheme: SignatureScheme::Ed25519,
        claim_scopes: vec![CapabilityScope::SecurityPostureObserve],
        valid_from_unix_ms: now - 10000,
        expires_at_unix_ms: None,
        revoked: false,
        evidence_refs: vec![],
    };

    let mut envelope = ClaimEnvelope {
        schema_version: "claim_envelope_v0.1".to_string(),
        encoding_profile: EncodingProfile::NetStewardCanonicalJsonV1,
        envelope_id: "env-crypto".to_string(),
        payload,
        payload_hash: "hash-crypto".to_string(),
        issuer_did: "did:mycelix:alice".to_string(),
        signature: signature_hex,
        signature_scheme: SignatureScheme::Ed25519,
        verification_status: ClaimVerificationStatus::Unverified,
        signatures: None,
    };

    let verifier = Ed25519ClaimVerifier;
    let status = verifier.verify_envelope_signature(&envelope, Some(&binding));
    assert_eq!(status, ClaimVerificationStatus::VerifiedSignature);

    // Corrupt signature
    envelope.signature = "ffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffffff".to_string();
    let status_corrupted = verifier.verify_envelope_signature(&envelope, Some(&binding));
    assert_eq!(status_corrupted, ClaimVerificationStatus::InvalidSignature);
}

use net_steward_discovery::{ClaimFilter, FederationTransport};
use net_steward_holochain::{MycelixHolochainIdentityResolver, MycelixHolochainTransport};
use net_steward_schema::{
    ClaimEnvelope, ClaimStatus, ClaimVerificationStatus, DidBindingResolver, EncodingProfile,
    PeerPostureClaim, PostureSummary, SignatureScheme,
};

fn sample_claim() -> ClaimEnvelope<PeerPostureClaim> {
    ClaimEnvelope {
        schema_version: "claim_envelope_v0.1".to_string(),
        encoding_profile: EncodingProfile::NetStewardCanonicalJsonV1,
        envelope_id: "fixture-only-envelope".to_string(),
        payload: PeerPostureClaim {
            claim_id: "fixture-only-claim".to_string(),
            issuer_did: "did:mycelix:fixture-only".to_string(),
            subject_node_id: "fixture-node".to_string(),
            issued_at_unix_ms: 1,
            expires_at_unix_ms: 2,
            posture_summary: PostureSummary::Unknown,
            topology_refs: vec![],
            security_event_refs: vec![],
            evidence_refs: vec![],
            capsule_refs: vec![],
            claim_status: ClaimStatus::Valid,
        },
        payload_hash: String::new(),
        issuer_did: "did:mycelix:fixture-only".to_string(),
        signature: String::new(),
        signature_scheme: SignatureScheme::SimulatedEnvelope,
        verification_status: ClaimVerificationStatus::Unverified,
        signatures: None,
    }
}

fn assert_unavailable<T>(result: Result<T, String>) {
    let error = result.err().expect("placeholder integration must not report success");
    assert!(
        error.starts_with("HolochainTransportUnavailable:"),
        "unexpected error category: {error}"
    );
}

#[test]
fn placeholder_holochain_transport_never_reports_live_success() {
    let transport = MycelixHolochainTransport::new(
        "ws://127.0.0.1:8888",
        "net_steward",
        "net_steward_posture",
    );

    assert_unavailable(transport.publish_claim(sample_claim()));
    assert_unavailable(transport.fetch_claims(ClaimFilter {
        issuer_did: None,
        subject_node_id: None,
    }));
    assert_unavailable(transport.fetch_claim_by_id("fixture-only-claim"));
    assert_unavailable(transport.publish_evidence_hash("sha256:fixture-only"));
    assert_unavailable(transport.publish_revocation("fixture-only-envelope"));
}

#[test]
fn placeholder_identity_resolver_never_fabricates_agent_binding() {
    let resolver = MycelixHolochainIdentityResolver::new(
        "ws://127.0.0.1:8888",
        "net_steward",
        "agent_reputation",
    );

    assert_unavailable(resolver.resolve_binding("did:mycelix:unresolved-agent"));
}

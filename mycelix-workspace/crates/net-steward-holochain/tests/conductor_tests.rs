//! Fail-closed tests for the current Holochain adapter seam.
//!
//! These are unit tests for the explicit unavailable boundary, not conductor
//! integration tests. A future live adapter must add separate real two-agent
//! tests with exact conductor, DNA, application identity and artifact hashes.

use net_steward_discovery::{ClaimFilter, FederationTransport};
use net_steward_holochain::{
    MycelixHolochainIdentityResolver, MycelixHolochainTransport,
};
use net_steward_schema::{
    ClaimEnvelope, ClaimStatus, ClaimVerificationStatus, DidBindingResolver, EncodingProfile,
    PeerPostureClaim, PostureSummary, SignatureScheme,
};

fn fixture_claim() -> ClaimEnvelope<PeerPostureClaim> {
    ClaimEnvelope {
        schema_version: "claim_envelope_v0.1".to_string(),
        encoding_profile: EncodingProfile::NetStewardCanonicalJsonV1,
        envelope_id: "fixture-only-not-a-live-record".to_string(),
        payload: PeerPostureClaim {
            claim_id: "fixture-only-not-a-live-claim".to_string(),
            issuer_did: "did:mycelix:fixture".to_string(),
            subject_node_id: "fixture-node".to_string(),
            issued_at_unix_ms: 1_700_000_000_000,
            expires_at_unix_ms: 1_700_000_060_000,
            posture_summary: PostureSummary::Unknown,
            topology_refs: vec![],
            security_event_refs: vec![],
            evidence_refs: vec![],
            capsule_refs: vec![],
            claim_status: ClaimStatus::Valid,
        },
        payload_hash: "fixture-payload-hash".to_string(),
        issuer_did: "did:mycelix:fixture".to_string(),
        signature: "fixture-signature-not-cryptographically-verified".to_string(),
        signature_scheme: SignatureScheme::SimulatedEnvelope,
        verification_status: ClaimVerificationStatus::Unverified,
        signatures: None,
    }
}

fn assert_unavailable(result: Result<(), String>, operation: &str) {
    let error = result.expect_err("unimplemented live I/O must never return success");
    assert!(error.contains("Unavailable"), "unexpected error: {error}");
    assert!(error.contains(operation), "error should identify operation: {error}");
    assert!(
        error.contains("no network I/O was attempted"),
        "error must not imply a remote operation occurred: {error}"
    );
}

#[test]
fn unimplemented_transport_writes_and_reads_fail_closed() {
    let transport = MycelixHolochainTransport::new(
        "ws://127.0.0.1:8888",
        "net_steward",
        "net_steward_posture",
    );

    assert_unavailable(transport.publish_claim(fixture_claim()), "publish_claim");
    assert_unavailable(
        transport.publish_evidence_hash("fixture-evidence-hash"),
        "publish_evidence_hash",
    );
    assert_unavailable(
        transport.publish_revocation("fixture-envelope-id"),
        "publish_revocation",
    );

    let fetch_error = transport
        .fetch_claims(ClaimFilter {
            issuer_did: None,
            subject_node_id: None,
        })
        .expect_err("unimplemented fetch must not synthesize a DHT response");
    assert!(fetch_error.contains("Unavailable"));
    assert!(fetch_error.contains("fetch_claims"));

    let lookup_error = transport
        .fetch_claim_by_id("fixture-only-not-a-live-claim")
        .expect_err("unimplemented lookup must not synthesize a DHT response");
    assert!(lookup_error.contains("Unavailable"));
    assert!(lookup_error.contains("fetch_claim_by_id"));
}

#[test]
fn unimplemented_identity_resolver_never_fabricates_a_binding() {
    let resolver = MycelixHolochainIdentityResolver::new(
        "ws://127.0.0.1:8888",
        "net_steward",
        "agent_reputation",
    );

    let error = resolver
        .resolve_binding("did:mycelix:unknown")
        .expect_err("unimplemented resolver must not fabricate an AgentPubKey or scopes");

    assert!(error.contains("Unavailable"));
    assert!(error.contains("no identity binding was asserted"));
}

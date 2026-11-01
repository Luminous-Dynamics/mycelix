use gcpn_evidence_graph_lifecycle::{
    validate_cap_lifecycle, CapMessage, CapMessageType, DigestVerification,
};

fn main() {
    let alert = CapMessage {
        sender: "synthetic-authority.example".into(),
        identifier: "fixture-alert-001".into(),
        sent_at: 100,
        message_type: CapMessageType::Alert,
        references: vec![],
        expires_at: Some(500),
        recorded_at: 101,
        wire_bytes: b"<alert>synthetic-v1</alert>".to_vec(),
        declared_digest: "sha256:externally-verified-fixture-digest".into(),
        digest_verification: DigestVerification::FixtureOracleAccepted,
    };
    let update = CapMessage {
        sender: "synthetic-authority.example".into(),
        identifier: "fixture-update-001".into(),
        sent_at: 200,
        message_type: CapMessageType::Update,
        references: vec![alert.extended_identifier()],
        expires_at: Some(600),
        recorded_at: 201,
        wire_bytes: b"<alert>synthetic-v2</alert>".to_vec(),
        declared_digest: "sha256:externally-verified-update-digest".into(),
        digest_verification: DigestVerification::FixtureOracleAccepted,
    };

    let audit = validate_cap_lifecycle(&[alert, update]);
    let report = audit.report;
    println!("GCPN bitemporal evidence graph lifecycle v0.1");
    println!("status: synthetic_fixture_oracle_example");
    println!("messages_seen: {}", report.messages_seen);
    println!("updates: {}", report.updates);
    println!("cancellations: {}", report.cancellations);
    println!("missing_references: {}", report.missing_references);
    println!("unknown_references: {}", report.unknown_references);
    println!("expired_references: {}", report.expired_references);
    println!("conflicting_identifiers: {}", report.conflicting_identifiers);
    println!("unverified_digests: {}", report.unverified_digests);
    println!("digest_mismatches: {}", report.digest_mismatches);
    println!("conflicting_payloads_preserved: {}", audit.conflicting_messages.len());
    println!("conformant: {}", report.is_conformant());
    println!("non_claim: this core does not parse CAP XML or calculate/verify cryptographic digests");
}

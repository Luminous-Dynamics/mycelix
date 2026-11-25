use mycelix_temporal_evidence_kernel::{
    TemporalAssertionV1, TemporalDimensionV1, TemporalEvidenceSetV1, TemporalPrecisionV1,
    TemporalQualityV1,
};

fn main() {
    let assertions = TemporalEvidenceSetV1 {
        assertions: vec![
            TemporalAssertionV1 {
                assertion_id: "grant-revoked-valid-from".into(),
                claim_id: "grant-revocation-claim".into(),
                subject_ref: "grant:001:revocation".into(),
                dimension: TemporalDimensionV1::ValidFrom,
                time_value: "2026-11-18T00:00:00Z".into(),
                precision: TemporalPrecisionV1::Second,
                quality: TemporalQualityV1::SourceAsserted,
                clock_profile: "governance-record-clock-v1".into(),
                source_authority_ref: "governance:grant-issuer-001".into(),
                evidence_ref: "source:revocation-effective-time".into(),
                assertion_ref: None,
                verifier_ref: None,
                knowledge_frontier_ref: None,
                supersedes_assertion_id: None,
                uncertainty_window: None,
            },
            TemporalAssertionV1 {
                assertion_id: "verifier-learned-revocation".into(),
                claim_id: "grant-revocation-claim".into(),
                subject_ref: "grant:001:revocation".into(),
                dimension: TemporalDimensionV1::VerifierKnowledgeAt,
                time_value: "2026-11-20T08:00:20Z".into(),
                precision: TemporalPrecisionV1::Second,
                quality: TemporalQualityV1::SourceAsserted,
                clock_profile: "mycelix-verifier-clock-v1".into(),
                source_authority_ref: "mycelix:finance-verifier-v1".into(),
                evidence_ref: "source:revocation-verifier-receipt".into(),
                assertion_ref: Some("grant-revoked-valid-from".into()),
                verifier_ref: Some("did:mycelix:verifier:finance-v1".into()),
                knowledge_frontier_ref: Some("checkpoint:finance-verifier-frontier-001".into()),
                supersedes_assertion_id: None,
                uncertainty_window: None,
            },
        ],
    };
    assertions.validate().expect("the synthetic temporal evidence set is structurally valid");

    let historical = assertions
        .evaluate_valid_at(
            "grant-revocation-claim",
            "did:mycelix:verifier:finance-v1",
            "2026-11-19T00:00:00Z",
            "2026-11-19T00:00:00Z",
            true,
        )
        .expect("historical query succeeds");
    println!("as-known-before-verifier-receipt: {:?}", historical.disposition);

    let current_knowledge = assertions
        .evaluate_valid_at(
            "grant-revocation-claim",
            "did:mycelix:verifier:finance-v1",
            "2026-11-19T00:00:00Z",
            "2026-11-21T00:00:00Z",
            true,
        )
        .expect("current-knowledge query succeeds");
    println!("as-known-after-verifier-receipt: {:?}", current_knowledge.disposition);
}

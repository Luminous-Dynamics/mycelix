use std::hint::black_box;
use std::time::Instant;

use mycelix_temporal_evidence_kernel::{
    TemporalAssertionV1, TemporalDimensionV1, TemporalEvidenceIndexV1, TemporalEvidenceScanV1,
    TemporalEvidenceSetV1, TemporalPrecisionV1, TemporalQualityV1,
};

const VERIFIER: &str = "did:mycelix:verifier:benchmark-v1";
const FROM: &str = "2026-11-18T00:00:00Z";
const TO: &str = "2026-11-22T00:00:00Z";
const KNOWN: &str = "2026-11-23T00:00:00Z";
const VALID_AT: &str = "2026-11-19T00:00:00Z";

fn fact(
    id: String,
    claim_id: &str,
    subject_ref: &str,
    dimension: TemporalDimensionV1,
    time_value: &str,
) -> TemporalAssertionV1 {
    TemporalAssertionV1 {
        assertion_id: id.clone(),
        claim_id: claim_id.into(),
        subject_ref: subject_ref.into(),
        dimension,
        time_value: time_value.into(),
        precision: TemporalPrecisionV1::Second,
        quality: TemporalQualityV1::SourceAsserted,
        clock_profile: "benchmark-source-clock-v1".into(),
        source_authority_ref: "benchmark-source-authority-v1".into(),
        evidence_ref: format!("benchmark-evidence:{id}"),
        content_digest: format!("sha256:{}", "0".repeat(64)),
        validity_interval_ref: None,
        assertion_ref: None,
        verifier_ref: None,
        knowledge_frontier_ref: None,
        supersedes_assertion_id: None,
        uncertainty_window: None,
    }
}

fn knowledge(id: String, claim_id: &str, subject_ref: &str, fact_id: &str) -> TemporalAssertionV1 {
    let mut assertion = fact(
        id.clone(),
        claim_id,
        subject_ref,
        TemporalDimensionV1::VerifierKnowledgeAt,
        KNOWN,
    );
    assertion.evidence_ref = format!("benchmark-receipt:{id}");
    assertion.assertion_ref = Some(fact_id.into());
    assertion.verifier_ref = Some(VERIFIER.into());
    assertion.knowledge_frontier_ref = Some(format!("benchmark-frontier:{id}"));
    assertion
}

fn workload(claim_count: usize) -> TemporalEvidenceSetV1 {
    let mut assertions = Vec::with_capacity(claim_count.saturating_mul(4));
    for index in 0..claim_count {
        let claim_id = format!("claim:{index:08}");
        let subject_ref = format!("grant:{index:08}");
        let from_id = format!("from:{index:08}");
        let to_id = format!("to:{index:08}");
        assertions.push(fact(
            from_id.clone(),
            &claim_id,
            &subject_ref,
            TemporalDimensionV1::ValidFrom,
            FROM,
        ));
        assertions.push(fact(
            to_id.clone(),
            &claim_id,
            &subject_ref,
            TemporalDimensionV1::ValidTo,
            TO,
        ));
        assertions.push(knowledge(
            format!("from-known:{index:08}"),
            &claim_id,
            &subject_ref,
            &from_id,
        ));
        assertions.push(knowledge(
            format!("to-known:{index:08}"),
            &claim_id,
            &subject_ref,
            &to_id,
        ));
    }
    TemporalEvidenceSetV1 { assertions }
}

fn main() {
    println!(
        "claims,assertions,queries,scan_build_us,index_build_us,scan_query_us,indexed_query_us,speedup"
    );
    for claim_count in [32_usize, 64, 128, 256] {
        let evidence = workload(claim_count);

        let scan_build_start = Instant::now();
        let scan = TemporalEvidenceScanV1::new(evidence.clone())
            .expect("generated evidence should validate");
        let scan_build = scan_build_start.elapsed();

        let index_build_start = Instant::now();
        let index = TemporalEvidenceIndexV1::new(evidence.clone())
            .expect("generated evidence should validate");
        let index_build = index_build_start.elapsed();

        let query_count = claim_count.min(16);
        let reference_start = Instant::now();
        let mut reference_results = Vec::with_capacity(query_count);
        for query in 0..query_count {
            let claim_id = format!("claim:{:08}", (query * 17) % claim_count);
            let result = scan
                .evaluate_valid_at(&claim_id, VERIFIER, VALID_AT, KNOWN, false)
                .expect("prevalidated scan query should succeed");
            reference_results.push(black_box(result));
        }
        let reference_elapsed = reference_start.elapsed();

        let indexed_start = Instant::now();
        for (query, expected) in reference_results.iter().enumerate() {
            let claim_id = format!("claim:{:08}", (query * 17) % claim_count);
            let result = index
                .evaluate_valid_at(&claim_id, VERIFIER, VALID_AT, KNOWN, false)
                .expect("indexed query should succeed");
            assert_eq!(&result, expected);
            black_box(result);
        }
        let indexed_elapsed = indexed_start.elapsed();

        let reference_ns = reference_elapsed.as_nanos().max(1) as f64;
        let indexed_ns = indexed_elapsed.as_nanos().max(1) as f64;
        let speedup = reference_ns / indexed_ns;
        println!(
            "{claim_count},{},{query_count},{},{},{},{},{speedup:.3}",
            evidence.assertions.len(),
            scan_build.as_micros(),
            index_build.as_micros(),
            reference_elapsed.as_micros(),
            indexed_elapsed.as_micros(),
        );
    }
}

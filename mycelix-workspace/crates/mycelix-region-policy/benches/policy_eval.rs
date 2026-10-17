use std::hint::black_box;
use std::time::Instant;

use mycelix_region_policy::{
    AgreementVerificationFailure, AgreementVerifier, BoundaryAction, ClassificationClaim,
    ClassificationVerificationFailure, ClassificationVerifier, DataClass, FederationAgreement,
    PlacementRule, ProfileVerificationFailure, ProfileVerifier, RegionalProfile,
    TransferRequest, VerifiedClassification, VerifiedFederationAgreement, VerifiedProfile,
    evaluate_transfer,
};

struct BenchmarkVerifier;
impl ProfileVerifier for BenchmarkVerifier {
    fn verify_profile(
        &self,
        profile: &RegionalProfile,
        _now_ms: u64,
    ) -> Result<u64, ProfileVerificationFailure> {
        // Measures only the already-verified policy decision path.
        // This test adapter is not a cryptographic verifier.
        Ok(profile.valid_until_unix_ms)
    }
}

struct BenchmarkAgreementVerifier;
impl AgreementVerifier for BenchmarkAgreementVerifier {
    fn verify_agreement(
        &self,
        agreement: &FederationAgreement,
        _now_ms: u64,
    ) -> Result<u64, AgreementVerificationFailure> {
        // Benchmark-only adapter; it is not a signature verifier.
        Ok(agreement.valid_until_unix_ms)
    }
}

struct BenchmarkClassificationVerifier {
    evidence_until_ms: u64,
}
impl ClassificationVerifier for BenchmarkClassificationVerifier {
    fn verify_classification(
        &self,
        _claim: &ClassificationClaim,
        _now_ms: u64,
    ) -> Result<u64, ClassificationVerificationFailure> {
        Ok(self.evidence_until_ms)
    }
}

fn profile(scope: &str, source_scopes: &[&str], destination_scopes: &[&str]) -> RegionalProfile {
    RegionalProfile {
        schema_version: 1,
        profile_id: format!("bench-{scope}"),
        profile_version: "1.0.0".to_owned(),
        scope_id: scope.to_owned(),
        valid_from_unix_ms: 1,
        valid_until_unix_ms: 10_000_000,
        rules: vec![PlacementRule {
            data_class: DataClass::Personal,
            allowed_actions: vec![BoundaryAction::Disclose],
            allowed_destination_scopes: destination_scopes.iter().map(|scope| (*scope).to_owned()).collect(),
            allowed_source_scopes: source_scopes.iter().map(|scope| (*scope).to_owned()).collect(),
            purpose_required: true,
            recipient_required: true,
            consent_required: false,
        }],
        content_digest: "a".repeat(64),
        signing_key_id: "benchmark-only".to_owned(),
        signature: vec![1; 64],
    }
}

fn percentile(sorted: &[f64], quantile: f64) -> f64 {
    let index = ((sorted.len() as f64 * quantile).ceil() as usize)
        .saturating_sub(1)
        .min(sorted.len() - 1);
    sorted[index]
}

fn main() {
    const BATCHES: usize = 1_000;
    const OPERATIONS_PER_BATCH: usize = 100;
    const NOW_MS: u64 = 2_000;

    let source = VerifiedProfile::verify(
        profile("community-a", &[], &["community-z"]),
        NOW_MS,
        &BenchmarkVerifier,
    ).expect("benchmark source fixture");
    let destination = VerifiedProfile::verify(
        profile("community-z", &["community-a"], &[]),
        NOW_MS,
        &BenchmarkVerifier,
    ).expect("benchmark destination fixture");
    let content_digest = "b".repeat(64);
    let classification = VerifiedClassification::verify(
        ClassificationClaim {
            resource_id: "record:benchmark".to_owned(),
            content_digest: content_digest.clone(),
            data_class: DataClass::Personal,
            valid_from_unix_ms: 1,
            valid_until_unix_ms: 10_000_000,
        },
        NOW_MS,
        &BenchmarkClassificationVerifier { evidence_until_ms: 10_000_000 },
    )
    .expect("benchmark classification fixture");
    let agreement = VerifiedFederationAgreement::verify(
        FederationAgreement {
            schema_version: 1,
            agreement_id: "benchmark-agreement".to_owned(),
            agreement_version: "1.0.0".to_owned(),
            source_profile_id: source.profile().profile_id.clone(),
            source_profile_digest: source.profile().content_digest.clone(),
            destination_profile_id: destination.profile().profile_id.clone(),
            destination_profile_digest: destination.profile().content_digest.clone(),
            valid_from_unix_ms: 1,
            valid_until_unix_ms: 10_000_000,
            allowed_data_classes: vec![DataClass::Personal],
            allowed_actions: vec![BoundaryAction::Disclose],
            allowed_purposes: vec!["credential-check".to_owned()],
            allowed_recipient_ids: vec!["service:education".to_owned()],
            content_digest: "c".repeat(64),
            signing_key_id: "benchmark-only".to_owned(),
            signature: vec![1; 64],
        },
        NOW_MS,
        &BenchmarkAgreementVerifier,
    )
    .expect("benchmark agreement fixture");
    let request = TransferRequest {
        action: BoundaryAction::Disclose,
        resource_id: "record:benchmark",
        content_digest: &content_digest,
        classification: &classification,
        agreement: &agreement,
        purpose: Some("credential-check"),
        recipient_id: Some("service:education"),
        consent: None,
        evaluated_at_unix_ms: NOW_MS,
    };

    let mut micros_per_operation = Vec::with_capacity(BATCHES);
    let mut allowed_total = 0usize;
    for _ in 0..BATCHES {
        let start = Instant::now();
        for _ in 0..OPERATIONS_PER_BATCH {
            let decision = evaluate_transfer(
                black_box(&source),
                black_box(&destination),
                black_box(&request),
            );
            if decision.allowed {
                allowed_total += 1;
            }
            black_box(decision);
        }
        let elapsed_ms = start.elapsed().as_secs_f64() * 1_000.0;
        micros_per_operation.push(elapsed_ms * 1_000.0 / OPERATIONS_PER_BATCH as f64);
    }

    micros_per_operation.sort_by(f64::total_cmp);
    println!("benchmark=regional_policy_evaluate_transfer");
    println!("operations={}", BATCHES * OPERATIONS_PER_BATCH);
    println!("allowed={allowed_total}");
    println!(
        "latency_us_per_op p50={:.3} p95={:.3} p99={:.3}",
        percentile(&micros_per_operation, 0.50),
        percentile(&micros_per_operation, 0.95),
        percentile(&micros_per_operation, 0.99),
    );
    println!("notes=policy-only; verifier, storage, network, and signature costs excluded");
}
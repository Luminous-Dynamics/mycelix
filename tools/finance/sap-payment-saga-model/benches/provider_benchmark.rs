use sap_payment_saga_model::provider::{
    ProviderAttemptClassV1, ProviderContractV1, ProviderOperationKeyV1,
};
use std::hint::black_box;
use std::time::{Duration, Instant};

const SAMPLE_COUNT: usize = 30;
const SAMPLE_WINDOW: Duration = Duration::from_millis(200);
const NOW_UNIX_MS: u64 = 2_000;

fn fixture() -> (ProviderContractV1, ProviderOperationKeyV1) {
    let contract = ProviderContractV1::adyen_documentation_baseline_v1("company:benchmark-account");
    let contract_digest = contract
        .canonical_digest()
        .expect("benchmark contract digest");
    let key = ProviderOperationKeyV1 {
        provider_id: contract.provider_id.clone(),
        profile_id: contract.profile_id.clone(),
        api_version: contract.api_version.clone(),
        account_scope: contract.account_scope.clone(),
        region: "eu".into(),
        idempotency_key: "idem-00000000-0000-4000-8000-000000000001".into(),
        request_digest: format!("sha256:{}", "a".repeat(64)),
        created_at_unix_ms: 1_000,
        contract_digest,
    };
    (contract, key)
}

fn measure(name: &str, mut operation: impl FnMut()) {
    for sample in 1..=SAMPLE_COUNT {
        let start = Instant::now();
        let mut iterations = 0_u64;
        while start.elapsed() < SAMPLE_WINDOW {
            operation();
            iterations += 1;
        }
        let elapsed = start.elapsed();
        let ns_per_op = elapsed.as_nanos() as f64 / iterations as f64;
        println!(
            "PROVIDER_V1 sample metric={name} sample={sample} iterations={iterations} elapsed_ns={} ns_per_op={ns_per_op:.2}",
            elapsed.as_nanos()
        );
    }
}

fn main() {
    let (contract, key) = fixture();
    let permit = key
        .qualify_retry(
            &contract,
            &key.account_scope,
            &key.region,
            &key.request_digest,
            ProviderAttemptClassV1::TransportTimeoutUnknown,
            NOW_UNIX_MS,
        )
        .expect("benchmark retry qualification");
    println!(
        "PROVIDER_V1_METADATA sample_count={SAMPLE_COUNT} sample_window_ms={} rust_profile=provider-contract-v1",
        SAMPLE_WINDOW.as_millis()
    );
    measure("key_validate", || {
        black_box(key.validate(black_box(&contract))).expect("valid operation key");
    });
    measure("retry_permit_qualify", || {
        black_box(
            key.qualify_retry(
                black_box(&contract),
                black_box(&key.account_scope),
                black_box(&key.region),
                black_box(&key.request_digest),
                ProviderAttemptClassV1::TransportTimeoutUnknown,
                NOW_UNIX_MS,
            )
            .expect("valid provider retry permit"),
        );
    });
    measure("retry_permit_revalidate", || {
        black_box(permit.validate_against(black_box(&key), black_box(&contract), NOW_UNIX_MS))
            .expect("valid provider retry permit");
    });
}

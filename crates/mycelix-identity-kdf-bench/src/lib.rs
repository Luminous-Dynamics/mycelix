//! Opt-in, synthetic-only benchmark for production identity KDF parameters.
//!
//! Run in a real browser with the standalone workflow. This crate is a
//! benchmark harness, not part of any production app. It only uses a fixed
//! synthetic passphrase and synthetic payload; receipts contain timings and
//! browser metadata, never keys, salt/IV, ciphertext, or user data.

#[cfg(all(test, target_arch = "wasm32"))]
mod browser_benchmark {
    use mycelix_leptos_core::identity_crypto::benchmark_production_passphrase_path;
    use serde::Serialize;
    use wasm_bindgen::JsValue;
    use wasm_bindgen_test::*;

    wasm_bindgen_test_configure!(run_in_browser);

    const SAMPLE_COUNT: usize = 15;
    const SYNTHETIC_PASSPHRASE: &str =
        "mycelix synthetic KDF benchmark passphrase - not for identity use";

    #[derive(Serialize)]
    struct BenchmarkReceipt<'a> {
        schema_version: &'static str,
        benchmark: &'static str,
        environment_user_agent: &'a str,
        iteration_count: u32,
        sample_count: usize,
        metric_units: &'static str,
        warmup_samples_excluded: bool,
        kdf_samples_ms: &'a [f64],
        kdf_min_ms: f64,
        kdf_median_ms: f64,
        kdf_p95_ms: f64,
        kdf_max_ms: f64,
        decrypt_samples_ms: &'a [f64],
        decrypt_min_ms: f64,
        decrypt_median_ms: f64,
        decrypt_p95_ms: f64,
        decrypt_max_ms: f64,
        secret_material_included: bool,
        qualification_status: &'static str,
    }

    fn percentile(values: &[f64], fraction: f64) -> f64 {
        let mut sorted = values.to_vec();
        sorted.sort_by(f64::total_cmp);
        let index = ((fraction * sorted.len() as f64).ceil() as usize)
            .saturating_sub(1)
            .min(sorted.len() - 1);
        sorted[index]
    }

    fn validate_samples(values: &[f64], expected_count: usize) {
        assert_eq!(values.len(), expected_count);
        assert!(
            values.iter().all(|value| value.is_finite() && *value > 0.0),
            "every elapsed sample must be finite and positive"
        );
    }

    #[wasm_bindgen_test]
    async fn record_production_pbkdf2_and_full_decrypt_timings() {
        let window = web_sys::window().expect("benchmark runs in an actual browser");
        let user_agent = window
            .navigator()
            .user_agent()
            .unwrap_or_else(|_| "unavailable".to_string());

        // The core crate is compiled here as a normal dependency, not with
        // cfg(test); its API also hard-checks the 600,000 production rounds.
        let samples = benchmark_production_passphrase_path(
            SYNTHETIC_PASSPHRASE,
            SAMPLE_COUNT,
        )
        .await
        .expect("production-parameter benchmark must complete");

        validate_samples(&samples.kdf_ms, SAMPLE_COUNT);
        validate_samples(&samples.decrypt_ms, SAMPLE_COUNT);
        assert_eq!(samples.pbkdf2_iterations, 600_000);

        let receipt = BenchmarkReceipt {
            schema_version: "0.1.0",
            benchmark: "mycelix-identity-pbkdf2-production",
            environment_user_agent: &user_agent,
            iteration_count: samples.pbkdf2_iterations,
            sample_count: SAMPLE_COUNT,
            metric_units: "milliseconds",
            warmup_samples_excluded: true,
            kdf_samples_ms: &samples.kdf_ms,
            kdf_min_ms: samples.kdf_ms.iter().copied().fold(f64::INFINITY, f64::min),
            kdf_median_ms: percentile(&samples.kdf_ms, 0.50),
            kdf_p95_ms: percentile(&samples.kdf_ms, 0.95),
            kdf_max_ms: samples.kdf_ms.iter().copied().fold(0.0, f64::max),
            decrypt_samples_ms: &samples.decrypt_ms,
            decrypt_min_ms: samples
                .decrypt_ms
                .iter()
                .copied()
                .fold(f64::INFINITY, f64::min),
            decrypt_median_ms: percentile(&samples.decrypt_ms, 0.50),
            decrypt_p95_ms: percentile(&samples.decrypt_ms, 0.95),
            decrypt_max_ms: samples
                .decrypt_ms
                .iter()
                .copied()
                .fold(0.0, f64::max),
            secret_material_included: false,
            qualification_status: "timing_baseline_only_not_security_qualification",
        };

        let json = serde_json::to_string(&receipt)
            .expect("non-secret benchmark receipt serializes");
        let line = format!("IDENTITY_KDF_BENCHMARK_RECEIPT={json}");
        web_sys::console::log_1(&JsValue::from_str(&line));
        // Also print through the wasm test harness so CI can extract exactly
        // one non-secret JSON line into an immutable artifact.
        println!("{line}");
    }
}

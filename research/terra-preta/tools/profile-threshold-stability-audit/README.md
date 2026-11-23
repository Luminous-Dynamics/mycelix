# Profile NSR threshold-stability audit v0.1

This standalone Rust CLI compares profile-level NSR report artifacts produced from the same normalized sensor profile at multiple soil-moisture response thresholds. It is a robustness audit, not another PF classifier.

## Run

    cargo test --manifest-path research/terra-preta/tools/profile-threshold-stability-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/profile-threshold-stability-audit/Cargo.toml -- \
      research/terra-preta/tools/profile-threshold-stability-audit/fixtures/synthetic-threshold-reports.json \
      /tmp/terra-preta-profile-threshold-stability.json

The output path must not already exist. The report binds to the SHA-256 of the exact threshold-report bundle bytes.

## Pipeline

For each declared response threshold, run the existing profile response extractor and profile-flow auditor against the same normalized profile input:

    cargo run --manifest-path research/terra-preta/tools/profile-response-extractor/Cargo.toml -- profile.json profile-001.json 0.01
    cargo run --manifest-path research/terra-preta/tools/profile-flow-audit/Cargo.toml -- profile-001.json report-001.json
    cargo run --manifest-path research/terra-preta/tools/profile-response-extractor/Cargo.toml -- profile.json profile-002.json 0.02
    cargo run --manifest-path research/terra-preta/tools/profile-flow-audit/Cargo.toml -- profile-002.json report-002.json

Bundle the resulting profile-flow reports into the `profile_reports` array. Keep the same source data, event, profile, onset method, sampling cadence and sensor roster; only the `response_threshold_m3_m3` may differ.

## Interpretation

- A sensor pair is `persistent_nsr_all_thresholds` only when the same deeper-first, interval-separated pair is present at every declared response threshold.
- `threshold_sensitive_ordering` means the same pair's observed ordering changes across thresholds.
- `partially_evaluable_nsr` and `not_evaluable_across_all_thresholds` retain pair absence at a threshold as missing comparability, not a negative result.
- Profile-level status distinguishes a persistent same-pair signature from NSR appearing at some but not all thresholds.

The tool checks a shared raw-source digest, event/profile/site identity, event window, cadence, onset method, declared sensor count, unique report digests, unique pairs, pair depths and strictly increasing thresholds. It does not reopen upstream artifact files or verify that their supplied SHA-256 values are truthful; it checks only the declarations carried by the input bundle.

## Scientific boundary

Threshold persistence is robustness only to the supplied theta-response threshold family. It does not remove shared sensor calibration error, baseline drift, saturation/frozen conditions, spatial aliasing or model misspecification. A robust NSR pair remains a profile-level response-order signature, not tracer-confirmed preferential flow. Absence of NSR does not rule out preferential flow.

Implementation tracker: [PF-NSR-ROBUST-001](https://github.com/Luminous-Dynamics/mycelix/issues/5566).

Reference: Nimmo et al. (2025), Vadose Zone Journal 24(2), e70017, DOI 10.1002/vzj2.70017. The review notes that soil-moisture response thresholds commonly span roughly 1–5% and that using multiple complementary methods can expose systematic bias.
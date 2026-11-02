# GCPN common-source comparison v0.1

This standalone Rust crate defines a strict comparison contract and matched-case availability report. It is an evaluation adapter foundation, not a forecast model or score engine.

## Core idea

A model comparison is only interpretable when candidate and reference outputs share a frozen comparison contract:

- the same predictand and units;
- the same temporal support, spatial grid and mask;
- the same climatology and evaluation period;
- the same verification source, release, artifact digest and observation lineage;
- the same as-of cutoff and eligible case frame;
- an explicit metric definition, scoring engine and configuration digest.

The manifest records the dataset release citation, retrieval time, declared license and redistribution policy. An unknown policy is never treated as permission. These are declarations for review, not a legal opinion.

## Paired case accounting

Every case row carries candidate, reference and common-verification availability with typed missing reasons. The report retains:
- eligible and excluded cases;
- candidate and reference marginal forecast availability;
- candidate-only, reference-only and both-missing cases;
- paired forecasts;
- verification availability and missing reasons;
- candidate-verifiable, reference-verifiable and paired-verifiable cases;
- paired forecasts that have no verification record.

Excluded case rows remain visible but do not enter the eligible denominator. Duplicate case keys fail closed. The report sorts rows into canonical order.

## Forecast product and metric requirements

Output manifests distinguish deterministic and ensemble products and record ensemble member counts. The metric manifest declares whether a scoring engine requires deterministic inputs, ensembles, equal member counts, an exact member count or a minimum member count.

For an intrinsic-skill claim, model training cutoffs must precede the evaluation period. Post-processing records carry their own fit-data digest and training cutoff; use of evaluation outcomes is rejected. Calibrated/post-processed products must be labelled separately from intrinsic forecast skill.

**No metric scores are calculated here.** Metric scoring is a subsequent layer. This crate verifies that the comparison inputs and case denominators are comparable before scores are attempted.

## Run

    cargo fmt --all -- --check
    cargo test --locked --offline
    cargo clippy --locked --offline --all-targets -- -D warnings
    cargo run --locked --offline

Rust 1.98.1 is pinned. The crate has no external dependencies and is isolated from the parent Cargo workspace.

## Qualification boundary

This is synthetic contract qualification only. It does not fetch, archive, redistribute or hash external dataset bytes. SHA-256 values are supplied by the manifest producer and checked for encoding/shape, not independently recomputed here. A future ingestion adapter must calculate digests from exact artifacts and attest the retrieval process.

The crate does not determine legal licensing rights, model forecast skill, meteorological quality, operational value or disaster impact. WeatherBench 2, WMO and operational NWP scorecards remain the appropriate external comparison references for actual forecast performance. GCPN's contribution is strict source/case/support comparability and transparent missingness accounting before any score is compared.

## Research basis

- WeatherBench 2: open like-for-like evaluation framework with a broad metric suite and model scorecards. Its displayed examples use different verifying analyses for some operational and ML model groups, illustrating why source identity and verification contracts must be made explicit.
- ECMWF Technical Memorandum 927 (2025): separates forecast reliability, resolution, information and noise, and stresses distinguishing intrinsic forecast skill from post-processing enhancements.
- GCPN outcome ascertainment and forecast information/noise packages provide additional independent qualification layers; this crate does not replace them.

## Status

Common-source comparison contract engine authored. Exact-head format, tests, Clippy and deterministic-output qualification are pending.

# Site-level PF prevalence auditor v0.1

This Rust CLI computes one event-level site prevalence record for the 2025 Li et al. NEON benchmark. It explicitly aggregates storms—not sensor rows, depth rows or plots—and keeps four distinct proxy estimands: VT under maximum Ksat, VT under median Ksat, VT under median-plus-standard-deviation Ksat, and NSR.

## Run

    cargo test --manifest-path research/terra-preta/tools/site-prevalence-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/site-prevalence-audit/Cargo.toml -- \
      research/terra-preta/tools/site-prevalence-audit/fixtures/synthetic-site-events.json \
      /tmp/site-prevalence-report.json

## Reproduced site probability definition

The paper's site probability is:

    count of qualifying rainfall events that triggered PF
    ----------------------------------------------------
    count of qualifying rainfall events with soil-moisture data available

Qualifying rainfall events have cumulative rainfall >= 2 mm. The normalizer applies that inclusive 2 mm rule and records each storm once per site. Multi-depth sensor, profile and plot evidence must be aggregated into a site-event record upstream.

## Missingness and identification bounds

Each method carries declared units, evaluable units and positive units. A site-event method outcome is:

- `positive` when at least one valid sensor/profile unit is positive, even if some other units are missing;
- `negative` only when every declared unit is evaluable and none is positive;
- `indeterminate` when no positive is seen but some declared units are not evaluable;
- `not_eligible` when rainfall is below 2 mm;
- `no_soil_moisture_data` when no usable profile evidence exists.

The report provides three distinct summaries per method: probability among evaluable site-events, a lower bound treating every indeterminate event as negative, and an upper bound treating every indeterminate event as positive. These bounds are identification bounds, **not confidence intervals**. The exact site-probability denominator remains the count of eligible events with moisture data, as in the paper; method coverage is separately shown so missingness cannot be hidden.

VT variants are never pooled. They correspond to maximum depth-weighted Rosetta Ksat, median depth-weighted Rosetta Ksat, and median plus one standard deviation. NSR is a separate method and is not part of the 2026 ResNet binary target.

## Scientific boundaries

- The input must have one row per unique site rainfall event. Do not duplicate a site-event by profile, plot, sensor or depth.
- The site probability is descriptive, not a causal effect, and event rows are temporally dependent. The independent unit for the 2025 Random Forest transfer task is the **site** (40 site rows), not the underlying sensor/event count.
- The source is NEON; the 2025 and 2026 papers are not independent field networks. Resolve shared event/site/sensor provenance before pooling or calling one external validation of the other.
- NSR and VT are soil-moisture-derived signatures. Neither is tracer-confirmed pathway ground truth.
- This audit computes descriptive site prevalence only. It does not implement the random forest, train/test partition, predictor processing, permutation importance or partial dependence.
# Trajectory-signature audit v0.1

This Rust CLI calculates event-window soil-moisture trajectory features for one sensor: maximum positive rate of rise, peak moisture, amplitude from the pre-event baseline, and cumulative positive increments. An optional, evidence-referenced maximum diffuse-rate threshold produces a trajectory-based proxy flag; without it, the tool reports signature metrics only.

## Run

    cargo test --manifest-path research/terra-preta/tools/trajectory-signature-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/trajectory-signature-audit/Cargo.toml -- \
      research/terra-preta/tools/trajectory-signature-audit/fixtures/synthetic-multi-peak.json \
      /tmp/terra-preta-trajectory-report.json

## Metrics

- Maximum positive rate of rise: max[(theta(t+dt)-theta(t))/dt], reported in m3/m3/hour with the interval timestamps.
- Peak theta and amplitude relative to the last pre-event sample.
- Cumulative positive rise: sum of successive positive theta increments; not a water volume or flux without a spatial support and control-volume model.
- Optional threshold comparison against a declared maximum diffuse-flow rise rate, with method and evidence reference required.

No smoothing is applied. The maximum-rate metric may be dominated by a single noisy increment; smoothing must be a separate, predeclared sensitivity analysis. Missing pairs are skipped but mark the report incomplete. Interpolated samples are included but counted and disclosed.

## Scientific boundary

Trajectory-based response features complement NSR and velocity-threshold methods. They do not independently establish a preferential pathway. An absent threshold yields no PF proxy classification. A supplied threshold is a model/measurement-conditioned proxy, not tracer ground truth. The 2025 Nimmo et al. review notes that maximum rate of rise can reduce the need to identify one exact onset and can accommodate multiple rising limbs, but threshold selection and measurement cadence remain material.

Reference: Nimmo et al. (2025), *Identifying preferential flow from soil moisture time series: Review of methodologies*, Vadose Zone Journal 24(2), e70017. https://doi.org/10.1002/vzj2.70017
# Conservative known-truth generator v0.1

This standalone Rust crate creates deterministic, conservative **two-reservoir** soil-water/tracer scenarios for testing data pipelines, mass-balance verifiers, response detectors, label separation and parameter-identifiability tools.

## Run

    cargo test --manifest-path research/terra-preta/tools/known-truth-generator/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/known-truth-generator/Cargo.toml -- \
      research/terra-preta/tools/known-truth-generator/fixtures/connected-macropore.json \
      /secure/truth/connected-truth.json \
      /tmp/model-visible-boundary-observations.json

## Model definition

Each 10-minute step adds rainfall to two well-mixed storage pools. The macropore input fraction partitions rainfall; finite storage capacities cause matrix-domain overflow to be recorded as surface runoff and macropore-domain capacity overflow to be recorded separately as an unclassified external loss. A declared inter-domain exchange fraction transfers water and its advected tracer mass from the more saturated pool to the other. First-order drainage removes matrix water to the lower boundary; macropore drainage is permitted only when the known-truth scenario says the pathway is connected. Potential evaporation removes matrix water but not conservative tracer mass.

Rainfall is in mm, which equals L/m². Tracer concentration is mg/L, so the input tracer mass is mg/m². The generator records every input, storage change, surface runoff, macropore capacity overflow, drainage and tracer transfer. Stepwise water and tracer balances must close within 1e-10 in the chosen units; no silent correction is applied.

## Scenario families

- `matrix_only`: no macropore rainfall partition, no connected macropore boundary, zero inter-domain exchange, and zero initial macropore storage.
- `connected_macropore`: macropore rainfall partition and direct macropore drainage to the lower boundary.
- `disconnected_macropore`: macropore storage exists but cannot drain directly to the lower boundary; it may exchange back to the matrix. Capacity overflow is recorded without assuming it is lateral flow.
- `dual_domain_exchange`: connected macropore pathway plus nonzero matrix/macropore exchange.

## Critical limits

This is **not** Richards' equation, a finite-element/finite-volume soil-profile solver, MACRO/RZWQM2/HYDRUS, a pore-network model or a field-calibrated simulator. It omits spatial vertical layering, capillarity, pressure gradients, dispersion, hysteresis, reactive chemistry, biological structure evolution and image-derived geometry. Parameters are per-step fractions, not hydraulic conductivity. Its value is controlled known truth and conservation, not realistic soil prediction.

Do not use its synthetic output to rank real amendments or recommend application rates. It is a first ground-truth layer that later observation operators and physically based kernels can build upon.

## Null-control interpretation

The `matrix_only` family rejects macropore inflow, connected drainage, nonzero inter-domain exchange, and nonzero initial macropore water or tracer. This prevents a nominal matrix-only negative control from secretly carrying or exchanging macropore mass.

The two-reservoir generator's macropore capacity overflow is **not** assigned a physical direction or mechanism. It is an accounting outflow needed to close the toy reservoir model. Do not call it lateral interflow, runoff or observed subsurface export without an explicit spatial boundary model.


## Separate privileged truth from model-visible observations

The CLI now writes two distinct outputs:

1. **Privileged truth manifest** — contains the scenario family, true connectivity, parameters, domain-specific storage/fluxes, tracer partition and stepwise conservation residuals. Keep this file outside model-fitting and scoring inputs.
2. **Model-visible observation file** — contains only the declared 10-minute forcing and aggregated total boundary outflow/tracer export. It does not expose matrix/macropore domain partition, domain storage, or domain-specific drainage.

The observation file states its operator (`synthetic_boundary_flux_aggregation_v0.1`) and measurement model (`perfect_noiseless_boundary_collection`), and carries the truth-manifest SHA-256 solely for provenance. This is a **noiseless observation operator**, not a realistic sensor model. It does not yet simulate theta probes, pressure transducers, cadence loss, noise, drift, collector recovery or censoring.

For an ML benchmark, provide only the observation file to the model and preserve the truth manifest in a separate evaluation service or access-controlled store. The model must not receive the target family, latent connectivity, hidden flux partition or true parameter vector. A later version should add a calibrated observation operator with uncertainty, missingness, sensor support and independent measurement error.

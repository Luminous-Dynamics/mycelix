# Profile response extractor v0.1

This Rust CLI turns normalized, multi-depth soil-moisture time series for one rainfall event into the exact v0.3 input contract consumed by `profile-flow-audit`. It closes the upstream gap between raw sensor observations and interval-censored nonsequential-response (NSR) profile diagnostics.

## Run

    cargo test --manifest-path research/terra-preta/tools/profile-response-extractor/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/profile-response-extractor/Cargo.toml -- \
      research/terra-preta/tools/profile-response-extractor/fixtures/synthetic-mixed-profile.json \
      /tmp/profile-audit-input.json 0.01
    cargo run --manifest-path research/terra-preta/tools/profile-flow-audit/Cargo.toml -- \
      /tmp/profile-audit-input.json /tmp/profile-audit-report.json

## Input contract

- One event, one profile, and the full declared sensor roster.
- Every sensor series must use the same explicit UTC sampling grid at 5–30 minute cadence.
- Each sensor must contain one directly observed pre-event baseline at exactly one sample interval before rainfall onset, plus every event-window bin through the declared observation end.
- Missing time bins must be represented explicitly as `missing` rows with null theta; silently omitted timestamps are rejected.
- Sensor condition must be declared `usable`, `saturated`, `frozen`, `sensor_fault`, or `unknown`. Non-usable sensors become indeterminate rather than false nonresponses.
- Water content is volumetric m3/m3. All values must be finite and within [0, 1].

## Response extraction

For a declared threshold delta, the baseline is the directly observed pre-event sample. The first directly observed post-onset sample with theta minus baseline greater than or equal to delta is the response. The output onset interval is `(last below-threshold sample, first above-threshold sample]`, exactly matching the profile-flow-audit interval-censoring contract.

A missing or interpolated sample before a crossing makes that sensor indeterminate in v0.1; it is not silently treated as observed. A saturated/frozen/faulted/unknown sensor is indeterminate because a theta-rise method cannot establish a valid nonresponse in those conditions. A valid complete observation window without a crossing is `not_detected`.

## Threshold sensitivity

Run the extractor separately at 0.01, 0.02, 0.03 and 0.05 m3/m3 (or a preregistered instrument/site-specific range), then feed every generated file into `profile-flow-audit`. Compare the resulting profile classes, pair counts, coverage and interval bounds. Do not choose the threshold after seeing which one produces the desired NSR class. The synthetic fixture intentionally changes classification as the threshold rises.

## Evidence boundary

The output is a sensor response-order input, not proof of preferential flow, a unique macropore, flux partition, vertical connectivity or tracer-confirmed bypass. Sensor spacing can miss narrow paths; lateral flow and heterogeneity can produce nonsequential responses. Keep proxy labels separate from tracer/geophysical adjudication.

Reference methodology: Nimmo et al. (2025), Vadose Zone Journal 24(2), e70017, https://doi.org/10.1002/vzj2.70017.
# Profile-level nonsequential response audit v0.1

This standalone Rust CLI computes the profile-level nonsequential response (NSR) signature for one precipitation event and one instrumented soil profile. It records all pairwise comparisons between responding sensors at distinct depths. It is a response-order diagnostic, not a pathway detector or tracer-ground-truth adjudicator.

## Run

    cargo test --manifest-path research/terra-preta/tools/profile-nsr-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/profile-nsr-audit/Cargo.toml -- \
      research/terra-preta/tools/profile-nsr-audit/fixtures/synthetic-nsr-event.json \
      /tmp/terra-preta-nsr-report.json

The report binds to the SHA-256 of exact input bytes and refuses to overwrite an existing output file.

## Input contract

One event, one site/profile, a precipitation start, an observation-window end, a 5–30 minute declared time resolution, an explicit volumetric response threshold and onset-detection method, and the full declared sensor roster. One installed sensor is a valid input and returns not-applicable rather than an error. Each sensor is `responded`, `no_response` or `indeterminate`. Responded sensors require a response-onset timestamp. Each profile event must declare the volumetric threshold and onset method used upstream; the auditor does not silently assume that all source datasets used the same onset definition. No-response and indeterminate sensors must have a null onset. Sensor IDs must be unique; sensors at the same depth are retained but are not compared as a vertical pair.

## Interpretations

- `nonsequential_response_signature`: at least one deeper sensor responded before a shallower sensor.
- `sequential_response_signature`: all distinct-depth responding pairs are ordered shallow-first, with no simultaneous tie and no indeterminate sensor.
- `ambiguous_simultaneous_response`: no deeper-first pair exists, but at least one pair shares the same recorded onset time.
- `not_applicable_insufficient_responders`: fewer than two sensors responded.
- `indeterminate_no_distinct_depth_pair`: at least two sensors responded, but all responding sensors were at the same depth.
- `indeterminate_incomplete_profile_without_observed_nsr`: the declared profile contains indeterminate sensors and no nonsequential pair was observed.

An observed NSR pair is retained when the profile is incomplete, but the report marks coverage as incomplete. A valid no-response sensor is not the same as an indeterminate measurement. Same-depth sensor pairs do not establish vertical ordering.

## Scientific limits

The 2025 Nimmo et al. review notes that NSR analysis commonly uses 5–30 minute measurements, and defines the signature as a deeper sensor responding before a shallower sensor. The review also notes that NSR tends toward false negatives: a sequential response cannot rule out bypass when sensors miss a pathway, are too sparsely spaced, or the pathway reaches sensors in order. Rapid PF and high matrix permeability can also make the signal appear sequential. Use NSR with VT, response trajectories, maximum rate-of-rise, spatial/geophysical observations, tracers and water balance. Agreement between NSR and VT is still not tracer confirmation.

Reference: Nimmo et al. (2025), *Identifying preferential flow from soil moisture time series: Review of methodologies*, Vadose Zone Journal 24(2), e70017. https://doi.org/10.1002/vzj2.70017
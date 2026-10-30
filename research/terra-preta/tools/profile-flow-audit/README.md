# Profile-level flow audit v0.2

This standalone Rust CLI evaluates profile-level nonsequential-response (NSR) signatures for one event and one sensor profile. It compares every responding sensor pair at distinct depths and retains pair ordering, simultaneous ties, valid nonresponses and indeterminate observations.

## Run

    cargo test --manifest-path research/terra-preta/tools/profile-flow-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/profile-flow-audit/Cargo.toml -- \
      research/terra-preta/tools/profile-flow-audit/fixtures/synthetic-mixed-profile.json \
      /tmp/terra-preta-profile-report.json

The report binds to the SHA-256 of exact input bytes and refuses to overwrite an existing report.

## Input contract

One event, site, plot, profile, precipitation start, observation-window end, sampling cadence (5–30 minutes), explicit volumetric response threshold, onset-detection method, source digest/evidence reference and the declared sensor roster. Each sensor has a unique identifier, depth and one of three statuses:

- `detected`: requires a response-onset timestamp within the declared event observation window.
- `not_detected`: a valid sensor did not respond; onset must be null.
- `indeterminate`: missing/invalid/unusable response evidence; onset must be null.

At least one installed sensor record is allowed. A profile with fewer than two responders is reported as not applicable, not rejected. Sensors at the same depth are retained but do not form vertical ordering pairs. All timestamps must be explicit UTC and aligned to the declared sampling grid.

## Profile classifications

- `nonsequential_response_detected`: at least one deeper sensor responded before a shallower sensor.
- `mixed_response_ordering`: both deeper-first and shallower-first pairs occur.
- `nonsequential_signature_incomplete_profile`: a deeper-first pair is observed, but one or more declared sensors are indeterminate.
- `sequential_response_only`: all comparable distinct-depth responding pairs are shallow-first.
- `sequential_and_simultaneous_response` / `ambiguous_simultaneous_response`: equal onset times are preserved as ties rather than ordered arbitrarily.
- `not_applicable_insufficient_responders`: fewer than two sensors responded.
- `indeterminate_no_distinct_depth_pair`: at least two sensors responded, but not at distinct depths.
- `indeterminate_incomplete_profile_without_nsr`: incomplete roster and no deeper-first pair was observed.

The report also records profile coverage status, pair counts and each pair's onset timestamps. Valid no-response sensors differ from indeterminate sensors. An observed NSR signature is retained when the profile is incomplete, but its coverage status remains incomplete.

## Interpretation boundary

NSR is a sensor-derived response-order signature, not proof of a unique pore pathway, tracer-confirmed preferential flow, flux partition or vertical-versus-lateral direction. Sequential response does not rule out preferential flow: sensors may miss narrow pathways, bypassed layers may be uninstrumented, and finite cadence may make rapid responses appear sequential. Nonsequential response can also reflect lateral routing, heterogeneity or measurement artefacts.

The 2025 Nimmo et al. review describes NSR as a deeper sensor responding before one or more shallower sensors and notes typical measurement intervals of 5–30 minutes. It identifies false-negative risks from sparse profiles and finite sampling cadence, and recommends combining NSR with velocity-threshold, other response signatures and independent geophysical/tracer/water-balance evidence.

Reference: Nimmo et al. (2025), *Identifying preferential flow from soil moisture time series: Review of methodologies*, Vadose Zone Journal 24(2), e70017. https://doi.org/10.1002/vzj2.70017
# Profile-level flow audit v0.3

This standalone Rust CLI evaluates profile-level nonsequential-response (NSR) signatures for one event and one sensor profile. It compares every responding sensor pair at distinct depths, retains valid nonresponses and indeterminate observations, and treats each sampled threshold crossing as an interval rather than an exact instant.

## Run

    cargo test --manifest-path research/terra-preta/tools/profile-flow-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/profile-flow-audit/Cargo.toml -- \
      research/terra-preta/tools/profile-flow-audit/fixtures/synthetic-mixed-profile.json \
      /tmp/terra-preta-profile-report.json

The report binds to the SHA-256 of exact input bytes and refuses to overwrite an existing report.

## Input contract

One event, site, plot, profile, precipitation start, observation-window end, sampling cadence (5–30 minutes), explicit volumetric response threshold, onset-detection method, source digest/evidence reference and the declared sensor roster. Each sensor has a unique identifier, depth and one of three statuses:

- `detected`: requires a first-above-threshold sample timestamp (`response_onset_utc`) within the observation window. The optional `response_onset_lower_bound_utc` records the last below-threshold sample; when omitted, the lower bound is derived by subtracting one declared sampling interval.
- `not_detected`: a valid sensor did not respond; onset must be null.
- `indeterminate`: missing/invalid/unusable response evidence; onset must be null.

At least one installed sensor record is allowed. A profile with fewer than two responders is reported as not applicable, not rejected. Sensors at the same depth are retained but do not form vertical ordering pairs. All timestamps must be explicit UTC and aligned to the declared sampling grid. A response interval must be ordered and no wider than one declared sampling interval.

## Profile classifications

- `nonsequential_response_detected`: at least one deeper sensor's latest possible response precedes a shallower sensor's earliest possible response (strictly separated intervals).
- `mixed_response_ordering`: both deeper-first and shallower-first pairs occur.
- `nonsequential_signature_incomplete_profile`: a deeper-first pair is observed, but one or more declared sensors are indeterminate.
- `sequential_response_only`: all comparable distinct-depth responding pairs are strictly shallow-first.
- `ambiguous_response_ordering_due_to_sampling_interval`: one or more response intervals overlap and no pair is robustly ordered.
- `sequential_response_with_ambiguous_pairs`: at least one pair is robustly sequential, while at least one other pair remains unresolved by sampling cadence.
- `sequential_and_simultaneous_response` / `ambiguous_simultaneous_response`: equal onset times are preserved as ties rather than ordered arbitrarily.
- `not_applicable_insufficient_responders`: fewer than two sensors responded.
- `indeterminate_no_distinct_depth_pair`: at least two sensors responded, but not at distinct depths.
- `indeterminate_incomplete_profile_without_nsr`: incomplete roster and no deeper-first pair was observed.

The report also records profile coverage status, NSR/sequential/tied/ambiguous pair counts, interval bounds for every pair, and minimum/maximum possible deep-first time differences. Overlapping intervals never count as NSR; exact ties are retained only when both onsets are explicitly represented as point times. Valid no-response sensors differ from indeterminate sensors. An observed NSR signature is retained when the profile is incomplete, but its coverage status remains incomplete.

## Interpretation boundary

NSR is a sensor-derived response-order signature, not proof of a unique pore pathway, tracer-confirmed preferential flow, flux partition or vertical-versus-lateral direction. Sequential response does not rule out preferential flow: sensors may miss narrow pathways, bypassed layers may be uninstrumented, and finite cadence may make rapid responses appear sequential. The interval-censoring rule deliberately favors fewer false positive NSR calls at the cost of some additional ambiguous pairs. Nonsequential response can also reflect lateral routing, heterogeneity or measurement artefacts.

The 2025 Nimmo et al. review describes NSR as a deeper sensor responding before one or more shallower sensors and notes typical measurement intervals of 5–30 minutes. It identifies false-negative risks from sparse profiles and finite sampling cadence, and recommends combining NSR with velocity-threshold, other response signatures and independent geophysical/tracer/water-balance evidence.

Reference: Nimmo et al. (2025), *Identifying preferential flow from soil moisture time series: Review of methodologies*, Vadose Zone Journal 24(2), e70017. https://doi.org/10.1002/vzj2.70017
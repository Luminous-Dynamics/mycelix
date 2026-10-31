# Profile-level preferential-flow signatures

This Rust CLI evaluates nonsequential response (NSR) across a single multi-depth soil profile for one declared event. It also reports continuous maximum moisture-rise rate and event increase per sensor. It is a soil-moisture signature auditor, not an independent pathway detector or hydraulic simulator.

## Run

    cargo test --manifest-path research/terra-preta/tools/profile-signature-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/profile-signature-audit/Cargo.toml -- \
      research/terra-preta/tools/profile-signature-audit/fixtures/synthetic-profile-event.json \
      /tmp/terra-preta-profile-signature-report.json

## Semantics and gates

- Requires at least two unique sensors at distinct depths, shared timestamps and common 5–30 minute regular cadence.
- All timestamps must be explicit UTC (zero offset). It rejects duplicate depths, duplicate sensor IDs, non-monotonic or irregular sampling, unsupported cadence and missing event coverage.
- Each sensor response is the first event-window sample at least the declared threshold above the immediately preceding sample before event start. Threshold is explicit input; 0.01 m3/m3 in the fixture is not a universal choice.
- NSR is positive when a deeper sensor responds before a shallower sensor. A tie is recorded as simultaneous within sampling resolution, not as NSR.
- A no-NSR result is reported only when at least two depths respond and all sensors are determinate. Fewer than two responding depths or incomplete coverage is indeterminate.
- Per-sensor maximum rate of rise is a continuous descriptive statistic across adjacent valid event samples. It is not turned into a PF class.
- Every report binds the exact input SHA-256 and preserves pair-level response order and time difference.

## Interpretation boundary

NSR can provide direct evidence of a nonsequential sensor response, but its absence does not establish absence of preferential flow: a pathway may bypass uninstrumented layers or miss point sensors. Fast moisture responses may also reflect piston displacement or rising water table. Independent tracer, geophysical, water-table and boundary-flux evidence remains necessary for physical pathway claims.

The fixture and tests are synthetic software controls only; they are not NEON observations, tracer validation, treatment efficacy or field recommendations.
# Profile-level flow audit v0.1

This standalone Rust CLI compares event-response onset times across sensors in one soil profile. It implements the nonsequential-response (NSR) diagnostic as an additional sensor-derived method; it does not identify unique pore geometry or certify preferential flow.

## Run

    cargo test --manifest-path research/terra-preta/tools/profile-flow-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/profile-flow-audit/Cargo.toml -- \
      research/terra-preta/tools/profile-flow-audit/fixtures/synthetic-mixed-profile.json \
      /tmp/terra-preta-profile-report.json

Input requires one event and all sensor response records from a single site/plot/profile. Each sensor carries an explicit status: detected, not_detected or indeterminate. Detected requires an onset time; the other two statuses must not have one.

The tool sorts sensors by depth and compares every unequal-depth pair that has two detected responses. A deeper onset earlier than a shallower onset is an NSR pair. It preserves sequential, nonsequential, simultaneous-at-resolution and mixed profile patterns. Missing or indeterminate responses do not count as evidence of matrix-only flow.

All timestamps are required to be UTC and aligned to the 10-minute sampling grid. Equal response times remain ties rather than being ordered arbitrarily. Results include the exact input SHA-256 and the output file is created without overwrite.

## Interpretation boundary

NSR and velocity-threshold (VT) diagnostics are complementary sensor methods, not independent physical truth. Nonsequential response can miss pathways that bypass installed sensors; sequential response does not prove that preferential flow was absent. A nonsequential event can still be caused by heterogeneity, lateral routing or measurement artefacts. Tracer/geophysical/water-table/lysimeter evidence is needed to adjudicate pathway and boundary-flux claims.

## Synthetic tests

The test suite covers deeper-before-shallower NSR, sequential response, mixed pair orderings, tied onset times, missing-data abstention, duplicate IDs, impossible response times and equal-depth sensors. These are software conformance cases, not field validation.
# Preferential-flow evidence triangulation audit v0.1

This Rust CLI records concordance/disagreement between sensor-level velocity-threshold and trajectory-threshold proxies, preserves the profile-level NSR signature, and separately records independent tracer/geophysical/water-table/lysimeter references. It deliberately does not calculate a majority vote or a weighted consensus across proxy methods.

## Run

    cargo test --manifest-path research/terra-preta/tools/pf-triangulation-audit/Cargo.toml
    cargo run --manifest-path research/terra-preta/tools/pf-triangulation-audit/Cargo.toml -- \
      research/terra-preta/tools/pf-triangulation-audit/fixtures/synthetic-method-evidence.json \
      /tmp/terra-preta-triangulation-report.json

The report binds to the SHA-256 of exact input bytes and retains the source digest plus the output digests/evidence references for the upstream method reports.

## Evidence levels and aggregation boundary

- Velocity threshold and trajectory threshold are sensor-level proxy methods. Their agreement/disagreement is recorded per sensor at matched depths.
- NSR is profile-level. Its signature is carried through unchanged rather than forced into a sensor-level voting scheme.
- Independent references can be conservative tracer, stable-isotope tracer, dye tracer, ERT, GPR, water table, lysimeter boundary flux or another declared method. Each record carries its own outcome, evidence reference and detection limit/spatial-temporal resolution.
- Physical claim status is derived from independent references only. Conflicting references remain conflicting. Proxy agreement alone leaves the physical claim not adjudicated.

This tool records method artifact SHA-256 strings as supplied; it does not open or independently verify the referenced reports. Upstream verifier runs and file presence must be checked separately. An `independent_reference` entry is only as strong as the method, sampling support, QA, recovery and blinded review documented by that evidence record.

## Interpretation

Sensor-level proxy outcomes are counted as positive agreement, negative agreement, disagreement, one method only, neither method evaluable, or indeterminate. No consensus classification is produced. The report never calls a proxy-majority result a confirmed preferential pathway.

The synthetic fixture includes intentional VT/trajectory disagreement and one independent tracer-positive reference. These are software-test values only, not measured soil observations.
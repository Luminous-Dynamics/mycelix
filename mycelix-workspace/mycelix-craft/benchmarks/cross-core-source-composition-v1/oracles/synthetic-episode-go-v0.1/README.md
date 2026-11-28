# Craft deterministic synthetic episode generator V0.1

Test-only Go utility that materializes the frozen multi-source decision episode mutation registry into independently digest-bound scenario bundles.

## Output guarantees
- Stable scenario ordering by ID.
- Fixed seed and frozen scenario registry.
- Raw base fixture and registry digests bound into every output bundle.
- Canonical transformed-episode digest (Go encoding/json deterministic map-key encoding under this declared profile; not an RFC 8785/JCS conformance claim).
- Named mutation and expected proof-obligation result preserved, including a signed-but-selectively-incomplete feed that actually omits EV-008.
- Duplicate and unknown mutation IDs rejected; no network or ambient clock dependency.

## Use
```sh
go run ./cmd/craft-synth-generate \
  -episode ../../fixtures/craft-synthetic-multi-source-episode-v0.1.json \
  -scenarios ../../fixtures/craft-synthetic-scenario-mutations-v0.1.json \
  -out generated-scenarios.json
```

## Local qualification
Go 1.23.2: package tests, race detector, vet passed; oracle package statement coverage 78.5%. This reflects the generator unit tests and small inline test fixture, not successful execution against the committed multi-source fixture. Hosted exact-head execution will run the actual committed episode and mutation registry and compare two independent generations byte-for-byte.

No production decision logic, physical data, source authentication, authority, or autonomous effect is implemented.

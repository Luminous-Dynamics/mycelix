# Carbon-origin and double-counting registry audit

Standalone Rust auditor for overlap between previously credited biochar CDR and a candidate ESM soil-organic-carbon stock-change claim.

## Run

```sh
cargo fmt --manifest-path research/terra-preta/tools/carbon-origin-registry-audit/Cargo.toml -- --check
cargo test --manifest-path research/terra-preta/tools/carbon-origin-registry-audit/Cargo.toml
cargo clippy --manifest-path research/terra-preta/tools/carbon-origin-registry-audit/Cargo.toml --all-targets -- -D warnings
cargo run --manifest-path research/terra-preta/tools/carbon-origin-registry-audit/Cargo.toml -- \
  research/terra-preta/tools/carbon-origin-registry-audit/fixtures/synthetic-carbon-origin-registry.json \
  /tmp/carbon-origin-report.json
```

## Accounting rules

- Match active/retired biochar CDR entries to their applied batch, application record and exact batch-passport SHA-256.
- Consider applications dated within the candidate SOC quantification period.
- Require reviewed footprint overlap-group status; unknown spatial relationship blocks a claim.
- Require declared SOC correction >= overlapping credited biochar CDR.
- Eligible residual = max(0, gross ESM SOC stock change - declared correction).
- Return the matched registry, application and batch passport identifiers/digests.
- A PyC result is recorded as total pyrogenic carbon / unresolved source only. It is not assigned to the new batch.

## Boundaries

This does not perform GIS intersection, estimate natural background PyC, analytically identify applied biochar, certify an ESM measurement, verify a real registry or issue a CDR credit. The spatial overlap group and all digests are supplied claims that need independent governance and verification.

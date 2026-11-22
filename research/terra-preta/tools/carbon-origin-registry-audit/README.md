# Carbon-origin and double-counting registry audit

Standalone Rust auditor for overlap between previously credited biochar CDR and a candidate ESM soil-organic-carbon stock-change claim.

Run commands:
- cargo fmt --manifest-path research/terra-preta/tools/carbon-origin-registry-audit/Cargo.toml -- --check
- cargo test --manifest-path research/terra-preta/tools/carbon-origin-registry-audit/Cargo.toml
- cargo clippy --manifest-path research/terra-preta/tools/carbon-origin-registry-audit/Cargo.toml --all-targets -- -D warnings

Accounting rules:
- Match active/retired biochar CDR entries to their applied batch and field record.
- Consider applications dated within the candidate SOC quantification period.
- Require reviewed footprint overlap-group status; unknown spatial relationship blocks a claim.
- Require declared SOC correction >= overlapping credited biochar CDR.
- Eligible residual = max(0, gross ESM SOC stock change - declared correction).
- A PyC result is recorded as total pyrogenic carbon / unresolved source only. It is not assigned to the new batch.

This does not perform GIS intersection, estimate natural background PyC, analytically identify applied biochar, certify an ESM measurement, verify a real registry or issue a CDR credit. The spatial overlap group and all digests are supplied claims that need independent governance and verification.

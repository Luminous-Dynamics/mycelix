# Federated Payment Fraud Signal Evaluator

Independent, effects-disabled Rust reference evaluator for the South African PEM-compatible fraud-signal contract.

## Run

```sh
cargo fmt --manifest-path tools/finance/sa-payment-fraud-signal-evaluator/Cargo.toml -- --check
cargo test --manifest-path tools/finance/sa-payment-fraud-signal-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/sa-payment-fraud-signal-evaluator/Cargo.toml -- --self-test
```

The evaluator validates the Draft 2020-12 corpus and computes source-status, signature/key, audience/purpose, pairwise identifier, origin-deduplication, score calibration, expiry and correction-propagation dispositions.

It does not contact a bank, join customer records, score transactions, create a blacklist, make legal findings, block accounts or move money.

## Non-negotiable boundaries

- A relay cannot promote a suspected report to confirmed.
- A model score is not a source-authorised factual finding.
- A recipient's local decision does not mutate source status.
- A public stable identifier or raw PII is not an acceptable subject reference.
- Common-origin reports are not independent evidence.
- Expired, unavailable or out-of-scope data is not a clean/green result.
- Corrections and withdrawals remain append-only and require downstream propagation.

## Claim ceiling

All fixtures are synthetic. A passing evaluator proves only that modeled provenance, status, purpose, privacy and correction rules reject seeded unsafe cases. It does not prove operational fraud detection, national service integration, production privacy, legal reporting compliance or SARB endorsement.

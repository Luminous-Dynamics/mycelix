# South African RTP-to-Payment Linkage Evaluator

Independent, effects-disabled Rust evaluator for the synthetic RTP message-correlation corpus.

## Run

```sh
cargo fmt --manifest-path tools/finance/sa-payment-rtp-link-evaluator/Cargo.toml -- --check
cargo test --manifest-path tools/finance/sa-payment-rtp-link-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/sa-payment-rtp-link-evaluator/Cargo.toml -- --self-test
cargo clippy --manifest-path tools/finance/sa-payment-rtp-link-evaluator/Cargo.toml --all-targets -- -D warnings
```

The evaluator validates the Draft 2020-12 schema and the typed relationship between request, response, payer instruction, pacs.008 transfer, pacs.002 status, beneficiary account credit and settlement finality.

It does not contact a bank, submit a payment, move money or claim a live South African participant mapping.

## Core boundaries

- A pain.014 accepted response does not itself establish a payment instruction.
- A payer instruction does not prove a pacs.008 transfer was sent.
- A pacs.002 status report describes an instruction; it is not itself beneficiary credit or settlement finality.
- A matching amount/reference is never sufficient to correlate a transfer to an RTP request.
- A duplicate message with the same transfer fingerprint is idempotent; a distinct transfer or conflicting request link is ambiguous and quarantined.

# South African Request-to-Pay Abuse Evaluator

Independent, effects-disabled Rust evaluator for the source-bound requester authenticity and abuse corpus.

## Run

```sh
cargo fmt --manifest-path tools/finance/sa-payment-rtp-abuse-evaluator/Cargo.toml -- --check
cargo test --manifest-path tools/finance/sa-payment-rtp-abuse-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/sa-payment-rtp-abuse-evaluator/Cargo.toml -- --self-test
cargo clippy --manifest-path tools/finance/sa-payment-rtp-abuse-evaluator/Cargo.toml --all-targets -- -D warnings
```

## Guarantees tested

- A sender session, displayed name or account-name match cannot be promoted into collect authority or invoice legitimacy.
- Payee mismatch, compromised credentials, unsupported claims, incomplete presentation and privacy violations produce explicit non-positive dispositions.
- Decline, query, report, pending and blocked actions never create payment instructions.
- Repeat-after-decline remains an abuse signal, not an invented universal prohibition.
- Source-backed limits are participant-specific; unknown limits cannot be converted to hard rejection.
- Small-merchant appeals remain reviewable rather than automatically becoming permanent exclusion.
- AI summaries and accessibility paths cannot omit required material fields.

The evaluator does not contact a bank, verify a real merchant, adjudicate debt, block a live account, disclose data or initiate payment.

All corpus cases are synthetic. A passing run demonstrates only conformance to this reference contract and seeded negative controls.


## Risk observation integrity

Every observation commitment is a SHA-256 digest over canonical compact JSON with `evidence_commitment` excluded, recursively key-sorted objects, preserved array order, UTF-8, and no whitespace. The evaluator recomputes and compares each digest before accepting the observation. This detects payload mutation only: a hash is not a signature, issuer authentication, proof of truth, or legal adjudication.


## Verification of Payee comparator

The corpus includes EPC VOP v1.1 (effective 20 September 2026) and matching guidance v2.0 (published 3 November 2026). The evaluator keeps match, close match, no match and verification-not-possible separate. A name match does not establish legal identity or collection authority; no-match/unavailable is disclosed while payer choice remains an explicit independent state in the EU comparator. These rules are not imported as South African PayShap requirements.

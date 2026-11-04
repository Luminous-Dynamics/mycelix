# South African Verification of Payee Evaluator

Independent, effects-disabled Rust reference evaluator for the source-qualified Verification of Payee corpus.

## Run

```sh
cargo fmt --manifest-path tools/finance/sa-payment-vop-evaluator/Cargo.toml -- --check
cargo test --manifest-path tools/finance/sa-payment-vop-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/sa-payment-vop-evaluator/Cargo.toml -- --self-test
```

The evaluator validates Draft 2020-12 structure plus source/profile references, participant/account/name/audience binding, provider authority/signature/key/currentness, alias generation and payer decision semantics.

It does not implement a universal fuzzy matcher. The responding PSP's source-qualified matching profile owns the actual Match/Close Match/No Match classification. The evaluator checks that its response is correctly scoped, typed and current, then prevents a weak response from being promoted into an instruction.

## Fundamental boundaries

- Alias resolution is not name matching.
- Name matching is not legal identity or ownership proof.
- Match is not payer consent or payment authority.
- Close Match requires a fresh check and new payer confirmation.
- No Match, Verification Not Possible, Not Requested, Unknown and Stale cannot silently authorise an instruction.
- A South African scheme-wide VoP service remains unqualified in this corpus.

## Claim ceiling

All fixtures are synthetic. Passing proves only that the frozen verifier contract rejects seeded response-binding, status-laundering and payer-authority mutations. It does not prove live bank coverage, legal ownership, fraud reduction, actual name-match accuracy, SARB endorsement or safe payment execution.

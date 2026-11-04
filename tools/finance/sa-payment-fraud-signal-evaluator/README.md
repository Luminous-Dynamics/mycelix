# South African Payment Fraud Signal Evaluator

Independent, effects-disabled Rust reference evaluator for the synthetic federated signal contract.

```sh
cargo fmt --manifest-path tools/finance/sa-payment-fraud-signal-evaluator/Cargo.toml -- --check
cargo test --manifest-path tools/finance/sa-payment-fraud-signal-evaluator/Cargo.toml
cargo run --manifest-path tools/finance/sa-payment-fraud-signal-evaluator/Cargo.toml -- --self-test
```

The evaluator validates source/profile references, jurisdiction-specific notification-rule profiles, subject-reference privacy, purpose/audience scope, evidence independence, typed status transitions, per-profile signal TTL, monotonic UTC history, immutable event-to-version/digest binding, expiry, correction, withdrawal, appeal state, disclosure minimisation, historical recipient inventory completeness, and recipient notification/acknowledgement/cache-invalidation receipts.

It does not score customers, infer criminality, query a live bank, create or update a national fraud registry, disclose customer data, or make payment decisions.

All fixtures use synthetic participant and subject tokens. The stacked #5817 corpus contains 39 scenarios, 71 events and 25 deterministic negative mutations. It includes positive controls for independent corroboration and source-confirmed status, plus complete, pending, excepted and incomplete recipient-propagation outcomes. POPIA section 24 and GDPR Article 19 are modelled as distinct jurisdictional profiles; no universal propagation SLA is invented. PASS is limited to the checked-in schema and semantic corpus; it does not cryptographically authenticate a provider source or establish lawful data-sharing authority.

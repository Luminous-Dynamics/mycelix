# PSI-002B3AQ — runner-neutral exact-source qualifier

Status: **PREPARED / NOT EXECUTED / NOT A PASS**

Exact product subject:

```text
1a5fdee47750c2f90e76e4e9dd4374603b56cfd7
```

This qualifier adds no hosted workflow and may establish only that the exact structural RFC 9577/9578 query-credit source compiles and its registered source corpus passes in one bound offline Rust environment.

It does not qualify any Privacy Pass cryptographic implementation, token issuer, replay database, atomic spend authority, anonymity property, rate-limit theorem, PSI theorem, or application authority.

## Offline gate

```text
cargo fmt --check --all
cargo test --offline --all-targets
cargo clippy --offline --all-targets --all-features -- -D warnings
```

No network fallback is permitted.

## Claim ceiling

Even a PASS records:

```text
structural_source_compiled = true
registered_tests_passed = true
privacy_pass_backend_qualified = false
token_cryptographically_verified = false
atomic_single_use_established = false
query_credit_granted = false
anonymous_rate_limit_established = false
enumeration_resistance_established = false
production_admission = false
application_authority = false
```

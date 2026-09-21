# PSI-002B3B1 r2Q — runner-neutral exact-source qualifier

Status: **PREPARED / NOT EXECUTED / NOT A PASS**

Exact product subject:

```text
be4fabe15a7f9180db13d85725a020566d3f9095
```

This qualifier establishes only the exact process-local nonce-bound spend reference source if the bound offline execution succeeds.

## Product surface

```text
mycelix-workspace/mycelix-core/libs/psi-query-credit-spend-core/Cargo.toml
mycelix-workspace/mycelix-core/libs/psi-query-credit-spend-core/README.md
mycelix-workspace/mycelix-core/libs/psi-query-credit-spend-core/src/lib.rs
```

It also binds the exact inherited B3A r2 structural-credit crate required by the path dependency.

## r2 replay ratchets

The qualifier requires:

```text
replay key includes token_nonce_sha256
replay key excludes token_sha256
same nonce + different token artifact -> same replay identity
different nonce -> different replay identity
```

and requires the 16-way concurrent race regression and exact process-local `Mutex<BTreeSet<String>>` store shape.

## Offline execution

The qualifier reconstructs the exact sibling B3A/B3B1 crates from canonical Git objects and runs from the spend crate:

```text
cargo fmt --check --all
cargo test --offline --all-targets
cargo clippy --offline --all-targets --all-features -- -D warnings
```

No network fallback is permitted.

## Claim ceiling

Even a successful execution may establish only:

```text
process_local_reference_source_compiled = true
registered_tests_passed = true
process_local_atomic_single_use_established = true
```

It remains false for token cryptographic verification, cryptographic nonce/challenge binding, durable/multi-process/crash-safe single use, real query credit, rate limiting, enumeration resistance, production admission and application authority.

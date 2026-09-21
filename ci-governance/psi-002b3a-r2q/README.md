# PSI-002B3A r2Q — runner-neutral exact-source qualifier

Status: **PREPARED / NOT EXECUTED / NOT A PASS**

Exact product subject:

```text
e32b54c86d602989955820fe1be5cbe88490e1e5
```

This qualifier establishes no Privacy Pass cryptographic theorem. A successful execution may establish only that the exact B3A r2 structural source compiled and its registered source tests/lints passed in the bound offline environment.

## Frozen product surface

```text
mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core/Cargo.toml
mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core/README.md
mycelix-workspace/mycelix-core/libs/psi-privacy-pass-credit-core/src/lib.rs
```

The lock binds the exact product commit/tree/parent, exact three product blob OIDs, exact qualifier source OIDs, the eleven-case source-test count, offline command vector, and authority ceiling.

## Static ratchets

The qualifier requires the corrected r2 distinctions:

```text
token_sha256                  = token artifact identity
token_nonce_sha256            = replay-subject observation
token_challenge_digest_sha256 = RFC token challenge-digest observation
```

It also requires stable RFC 9578 token codes 0x0001/0x0002, `AtomicSingleUseRequired`, `ReadyForBackendVerification`, and explicit false methods for cryptographic token verification, nonce binding, challenge-digest binding, unspent state, atomic consumption, query credit, rate limiting, enumeration resistance, and application authority.

A caller-controlled `unspent`, `consumed`, or `spent` authority field is forbidden.

## Offline execution

The exact crate is reconstructed from canonical Git objects and run with:

```text
cargo fmt --check --all
cargo test --offline --all-targets
cargo clippy --offline --all-targets --all-features -- -D warnings
```

No network fallback is permitted. Missing cached dependencies are a truthful FAIL.

## Receipt ceiling

Even a PASS records:

```text
authority_scope = StructuralRfcSemanticsOnly
structural_source_compiled = true
registered_tests_passed = true
privacy_pass_backend_qualified = false
token_cryptographically_verified = false
token_nonce_cryptographically_bound = false
challenge_digest_cryptographically_bound = false
atomic_single_use_established = false
query_credit_granted = false
production_admission = false
application_authority = false
```

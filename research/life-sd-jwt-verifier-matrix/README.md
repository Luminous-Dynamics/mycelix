# LIFE-SD-JWT-VERIFY-MATRIX-001 — Verifier decision matrix

This executable is the next step after the published disclosure-digest tests in [PR #5496](https://github.com/Luminous-Dynamics/mycelix/pull/5496) and implements the initial research scope in [issue #5497](https://github.com/Luminous-Dynamics/mycelix/issues/5497).

## Paradigm shift: split cryptographic validity from protocol and application policy

A credential should not be judged by one boolean called "valid". The harness exercises the real issuer → holder-selective-disclosure → verifier functions in `affinidi-sd-jwt 0.1.5` and separately tests single-use nonce consumption. It reports a fixed case ID and expected/actual decision for each scenario.

## Frozen cases

- Positive control: selected `given_name` is recovered while `family_name` remains undisclosed.
- Wrong issuer key and issuer signature tampering.
- Duplicate disclosure and a disclosure issued under a different credential.
- Required key binding with no KB-JWT.
- Valid key-bound presentation, signed wrong-`typ` and wrong-`cnf.jwk` holder key cases (raw library result versus hardened policy), fresh/stale/future KB-JWT `iat`, wrong nonce, wrong audience, absent caller-provided holder verifier and wrong holder key.
- Mutating the disclosure set while keeping the old KB-JWT, which should invalidate the `sd_hash` binding.
- Same-nonce replay behavior. A stateless verifier can accept a cryptographically valid presentation again when the same nonce is supplied; one-time nonce consumption is therefore a separate application policy and must be enforced atomically by the deployment.

## Test key limitation

The crate's `_test-utils` HMAC signer/verifier is **not suitable for production**, uses symmetric test keys, and is only used to create deterministic trust relationships for behavioral tests. The synthetic `cnf.jwk` is not a general-purpose public-key resolver. Production must bind the holder verifier to the issuer-authenticated `cnf.jwk`, enforce allowed algorithms, and validate the request/session context.

The matrix pins the registry release `affinidi-sd-jwt 0.1.5`, verifies the signed KB-JWT through that release API first, then applies the harness’s fixed-clock freshness and `typ=kb+jwt` profile policies to the authenticated token. The wrong-`typ` negative control records two outcomes: the raw pinned verifier API accepts a correctly signed token with another `typ`, while the hardened benchmark policy rejects it. A parallel key-mismatch control creates a KB-JWT signed by an attacker-controlled test key while the issuer-signed `cnf.jwk` still names the legitimate holder key; the raw API accepts when the caller supplies that attacker verifier, while the hardened policy rejects the mismatch. This demonstrates the documented caller contract in a synthetic octet-key fixture, not a general JWK resolver. (The `KbFreshness` helper exists on the newer upstream workspace branch but is not available in the published 0.1.5 API, so the test intentionally does not depend on it.) The matrix covers in-window, stale, and excessive-future timestamps. The pinned library represents parsed disclosures with publicly mutable fields. A source-level probe shows that mutating the cached `claim_value` can diverge from the encoded disclosure; the harness therefore canonicalizes through `serialize()` plus `SdJwt::parse()` at its trust boundary. This is an integration hardening rule, not a claim that an attacker can mutate arbitrary in-process memory over the wire. The `ConcurrentSingleUseNonceLedger` uses a mutex to model atomic in-process consumption and namespaces the same nonce by verifier identity. A 64-thread barrier test requires exactly one winner for the same verifier/nonce pair. This is not distributed replay protection: production still needs atomic, expiry-aware, verifier-scoped nonce consumption in the durable transaction store and multi-process/multi-region concurrency tests.

## Run

```sh
cargo +1.96.0 fmt --manifest-path research/life-sd-jwt-verifier-matrix/Cargo.toml --all -- --check
cargo +1.96.0 test --manifest-path research/life-sd-jwt-verifier-matrix/Cargo.toml
cargo +1.96.0 clippy --manifest-path research/life-sd-jwt-verifier-matrix/Cargo.toml --all-targets -- -D warnings
cargo +1.96.0 run --release --manifest-path research/life-sd-jwt-verifier-matrix/Cargo.toml
```

The workflow checks out the exact PR SHA, pins Rust and action revisions, runs format/tests/strict Clippy, repeats the CSV report and compares bytes, then retains a receipt, manifest and generated Cargo.lock as artifacts. Until the lockfile is committed and exact-head CI passes, this stays a bootstrap benchmark.

## Normative traceability

See [`rfc9901-requirement-map.json`](rfc9901-requirement-map.json) for the explicit mapping from RFC 9901 clauses to test IDs, application-policy checks, and uncovered requirements. A row marked `application_policy`, `partially_tested`, or `not_covered` is not conformance evidence. In particular, algorithm allowlisting, complete JWT validity, general JWK resolution, distributed replay storage, and official conformance vectors remain out of scope.

## Claim ceiling

This matrix is a synthetic implementation behavior harness using one pinned crate and test-only HMAC. It is not full RFC conformance, OpenID4VP/HAIP conformance, production cryptographic security, automatic JWK-to-verifier validation, distributed replay protection, anonymity, differential privacy, or a performance/SOTA comparison. Protocol correctness, task utility, privacy leakage and operational cost remain separate benchmarks.

# PSI-002AQ — runner-neutral exact-source qualifier

Status: **PREPARED / NOT EXECUTED / NOT A PASS**

This qualifier is an exact one-commit child of PSI-002A product subject:

```text
4cd3bbc47c27a3f11df7b3308ea0888adbf2322e
```

It adds no GitHub Actions workflow and consumes no hosted-runner capacity.

## Qualification theorem

A real successful execution may establish only:

```text
exact PSI-002A source identity
+ exact canonical PEC dependencies
+ exact voprf 0.5.0 dependency declaration
+ isolated offline Rust build
+ registered synthetic/adversarial test corpus PASS
+ warnings-denied Clippy PASS
+ checkout immutability
    -> exact synthetic experiment source executed successfully
```

It does **not** establish:

```text
RFC 9497 implementation independently qualified
PSI cryptographic security
malicious-secure PSI
enumeration resistance
anonymous transport
registry authenticity/freshness
production admission
application authority
```

## Exact source surface

Product files:

```text
mycelix-workspace/mycelix-core/libs/psi-voprf-experiment/Cargo.toml
mycelix-workspace/mycelix-core/libs/psi-voprf-experiment/README.md
mycelix-workspace/mycelix-core/libs/psi-voprf-experiment/src/lib.rs
```

The lock also binds the exact inherited PEC core and canonical PEC-002A protocol-profile blobs required to materialize an isolated three-crate workspace.

## Offline execution

The qualifier runs:

```text
rustc --version
cargo --version
cargo fmt --check --all
cargo test --offline --workspace
cargo clippy --offline --workspace --all-targets -- -D warnings
```

No network fallback is permitted. Missing cached dependencies are a truthful FAIL.

## Static experiment probes

Before Rust execution the qualifier requires, among other invariants:

- exact `voprf = "=0.5.0"` pin;
- exact canonical PEC-002A subject binding;
- `QualificationState::Experimental`;
- synthetic-only namespace enforcement;
- service + session domain separation;
- explicit online-enumeration demonstration;
- VOPRF proof-failure regression;
- maximum query-set bound;
- duplicate-canonical-identifier rejection;
- explicit false claim-ceiling methods;
- no network/Holochain runtime dependency in the experiment crate.

Static source inspection remains source evidence, not runtime or cryptographic proof.

## Receipt ceiling

Even a PASS records:

```text
authority_scope = ExperimentalExecutionOnly
source_compiled = true
registered_tests_passed = true
voprf_backend_qualified = false
psi_security_established = false
enumeration_resistance_established = false
client_anonymity_established = false
production_admission = false
application_authority = false
```

The receipt must be written outside the checkout.

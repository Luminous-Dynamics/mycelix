# CrossDomainDisruptionV1 Rust fixture validator

This isolated Cargo workspace provides a Rust-first structural preflight for the
public synthetic fixture package associated with Mycelix #2942 / #4907.

## Run

From the repository root:

```sh
cargo test --locked --manifest-path scripts/ops_intel/rust-validator/Cargo.toml
cargo run --locked --manifest-path scripts/ops_intel/rust-validator/Cargo.toml
```

An alternate fixture directory may be supplied as the CLI's single argument.

## Scope

The validator rejects duplicate JSON object keys, trailing data, malformed JSON,
and inputs exceeding declared size/shape limits. It checks source/artifact/
subject references, frontier ancestry, observation/receipt order and frontier
cutoffs, named supplier-facility coverage, temporal-reconciliation restrictions,
simulation-only attempt/effect separation, mock-permit negative controls,
protected-field omission, and predicate/mutation identifiers.

Closed Serde records using deny_unknown_fields currently cover the F0 frontier,
candidate records, protected-field records, F2 attempt, and F2 mock-authority
cases. The validator also pins the expected identity sets for the fixture's
domains, subjects, sources, artifacts, observations, coverage records, candidate
inventory, F1/F3 deltas, 20 predicates, and 23 mutation descriptors; a same-count
substitution no longer passes as an unchanged fixture. The remaining fixture record families use explicit structural/reference
checks over the parsed JSON tree; this is a structural preflight, not yet a
complete typed domain schema.

The exact-name evaluator-key blacklist is only a narrow tripwire. It is not a
general information-flow proof and cannot prove that an arbitrary, innocuously
named hidden value was not copied into solver-visible data. The solver-visible
fixture must stay physically distinct from any evaluator-only package; the
independent verifier must not open evaluator-only data during conformance checks.

The isolated Cargo workspace keeps this tooling crate from becoming an implicit
member of Mycelix's broader product workspace.

## Nonclaims

This is **fixture-package structural preflight only**. It does not implement the
OPS-INTEL-001 canonical byte protocol, produce projection identity hashes,
validate operational truth, evaluate forecast quality, generate decisions, or
grant execution authority. It is not the independent semantic oracle specified
by #2950: that verifier must use a separate implementation path and independently
check the frozen protocol once #2946's byte-level rules are approved.

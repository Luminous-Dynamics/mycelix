# DJCC V0.1 seed oracle

This module contains an independently authored Go expectation table for the initial synthetic Decision Justification Composition Certificate (DJCC) seed manifest.

Run from this directory:

```sh
go test ./...
go test -race ./...
go vet ./...
```

## What is checked

The oracle checks the registry against its own expected disposition table:
- all eight frozen initial seed IDs occur exactly once;
- the eight stage IDs and every per-stage expected disposition are present;
- composition dispositions and five-dimensional claim ceilings match independent expected outcomes;
- each case is synthetic-only and has distinct positive/negative control references;
- hard-safety mutations have explicit operator and target;
- positive controls do not carry a mutation.

The hard-coded Go expectation table is not generated from the JSON manifest. This guards against accidental edits to the manifest silently changing the expected outcome.

## Claim boundary

This is **seed-registry validation**, not a candidate DJCC verifier, certificate cryptographic verifier, source-trust proof, owner-universe completeness proof, policy evaluator, authorization service, or production assurance result. The eight seed references are symbolic until their exact synthetic fixture bytes are frozen and hashed.

The next implementation increment is a Rust candidate verifier and byte-level canonicalization vectors, with a separate cross-language comparison. Candidate and oracle must not share expected-result logic.

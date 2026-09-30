# Integral Interop 1 — Cross-Layer Conformance

**Status:** ReferenceModelOnly.

This contract makes the mutation boundary executable across the full reference chain:

**OAD semantic selection → selected OAD commitment → D6X closure identity → D6W input commitment → D6W derivation commitment.**

The Integral developer guide describes the OAD → COS Certified Design Package as the authoritative design input for a COS production plan, including labor steps, skill requirements, material quantities, ecological coefficients, and lifecycle assumptions. This repository deliberately freezes a narrower, explicitly named semantic projection rather than treating the public pseudocode as a ratified wire schema.

## Classification

Every mutation tested here must fall into one of three classes:

1. **Identity-preserving** — the mutation is outside the frozen semantic boundary; selected OAD, D6X, and D6W identities remain stable.
2. **Identity-changing** — the mutation changes selected semantics; the change propagates through OAD commitment, D6X closure identity, D6W input, and D6W derivation.
3. **Invalid / no identity** — the mutation violates the selected semantic contract; validation rejects it and no downstream semantic identity is produced.

A fourth operational case is tested explicitly:

4. **Required dependency blocked** — a required D6P receipt is absent. D6X records a blocked semantic boundary, but D6W refuses to consume it. Adding the required current receipt changes D6X identity and permits D6W construction.

## Executable invariants

- Certification metadata mutation → no semantic/D6W identity change.
- Selected production-step mutation → all downstream identities change.
- Irrelevant FRS/COS candidate noise → no semantic/D6W identity change.
- Fractional selected BOM quantity → fail closed; no semantic identity.
- Required D6P receipt missing → D6X blocked; D6W input absent.
- Required D6P receipt present → D6X becomes complete and D6W input/derivation become available.
- Runtime retrieval evidence → audit certificate may change, but closure identity and D6W input remain stable.

The implementation is intentionally a conformance harness, not an assertion that Integral has adopted these exact Rust types, field names, or hash domains.


## Declarative corpus

The executable mutation matrix is also represented as
`mycelix-manufacturing/crates/cos_conformance/testdata/integral_interop_1_cross_layer_vectors.json`.

Each vector declares:

- `id`: stable mutation identifier;
- `operation`: `set`, `candidate-noise`, `require-d6p-receipt`, or `runtime-evidence`;
- `path`: mutation target, or `$` for whole-fixture operations;
- `value`: operation-specific value;
- `expected`: one of `identity-preserving`, `identity-changing`, `invalid-no-identity`, `blocked-no-d6w`, `dependency-satisfied`, or `audit-only`.

The test runner validates the corpus vocabulary and executes each supported semantic mutation. Runtime evidence is deliberately handled by the dedicated audit-boundary test because it mutates the D6X certificate rather than the source OAD fixture.

This makes the mutation matrix reviewable as data while retaining Rust-level assertions for the cryptographic propagation boundary.

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
- `expected`: an object containing the validity result, expected D6X status, and exact per-boundary delta booleans for OAD semantic identity, D6X identity/certificate, and D6W input/derivation.

The source-level conformance suite validates the corpus schema and the declared blocked/complete D6X→D6W boundary. The corpus is a declarative mutation matrix; the individual semantic mutations remain exercised by the corresponding Rust tests rather than being dynamically interpreted from this JSON file. Runtime evidence is deliberately handled by the dedicated audit-boundary test because it mutates the D6X certificate rather than the source OAD fixture.

This makes the mutation matrix reviewable as data while retaining Rust-level assertions for the cryptographic propagation boundary. The companion Rust golden-vector test additionally freezes exact commitments for the baseline, missing-dependency rejection, and dependency-satisfied paths.


## Exact propagation matrix

The machine-readable corpus now records the expected delta at each identity boundary, rather than only a coarse classification. Each valid vector declares whether it changes:

- the selected OAD semantic commitment;
- the D6X candidate-independent closure identity;
- the D6X certificate/audit commitment;
- the D6W input commitment;
- the D6W derivation commitment;
- the D6X closure status.

This makes an important distinction executable:

- **semantic identity** answers whether the qualified dependency boundary changed;
- **certificate/audit identity** may change when candidate or runtime evidence changes without changing semantic identity;
- **D6W** is fail-closed and must never consume an incomplete D6X closure.

For the current ReferenceModelOnly fixture, candidate noise and runtime resolution evidence are intentionally audit-only, while a required-but-missing D6P receipt changes the D6X closure identity and blocks D6W consumption.

The corpus therefore checks the stronger invariant:

> every mutation propagates exactly through the declared dependency boundary, and no farther.

The Integral developer guide describes OAD → COS as a data contract in which the certified design package is the authoritative source for the COS production plan; the conformance fixture remains a local reference model rather than a claim about a ratified Integral wire schema. 


## Graph-boundary mutation coverage

The corpus now reaches below the OAD projection boundary into the qualified graph itself. It exercises:

- selected root-node commitment changes;
- selected semantic-edge commitment changes;
- selected semantic-edge removal;
- selected semantic-edge kind substitution with a non-dependency edge kind;
- irrelevant candidate-node commitment changes;
- irrelevant candidate-edge commitment changes.

Selected graph mutations must propagate through D6X and D6W even when the originating OAD semantic commitment is unchanged. Irrelevant graph mutations must be visible only to the D6X certificate/audit commitment.

This deliberately separates the **source semantic boundary** from the **qualified graph boundary**: a graph can change without the upstream OAD semantic projection changing, and the test must prove whether that graph mutation is inside or outside the declared D6X dependency policy.

The public Integral documentation currently describes the OAD → COS Certified Design Package as the contract that carries the certified design's production-plan information, while the project's technical-specification page labels the Certified Design data structure as DRAFT and the OAD → COS interface as PENDING. This is why these vectors remain explicitly ReferenceModelOnly rather than being presented as a ratified Integral protocol. citeturn0search22turn0search3

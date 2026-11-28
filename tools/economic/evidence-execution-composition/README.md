# DJCC V0.1 independent Go oracle

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

## Composition profiles and root claim ownership

`DJCC_PROFILES_V0_1.json` defines two profiles:
- `mycelix.djcc.full-evidence-execution.v0.1`: requires authentication, authority, completeness, effect and truth, with profile-owned stage sets for each claim.
- `mycelix.djcc.historical-decision-only.v0.1`: requires authentication, authority, completeness and truth, and marks execution/outcome explicitly NotApplicable. It cannot make a physical effect or outcome claim.

Every stage edge declares a non-empty, sorted, unique `required_claims` list. This is deliberately more granular than requiring the parent stage to be entirely Verified. For example, semantic projection can consume the established Authentication claim of a source-trust result while that source-trust stage remains Indeterminate about issuer participation Completeness. A query that needs complete coverage separately requires Completeness from the relevant frontier/source/projection owners.

The profile maps final root claims to the stages that own them. The root disposition is derived from the profile's required claim set, not by turning every unrelated stage abstention into an all-or-nothing failure. A NotApplicable stage must have all its claim dimensions marked NotApplicable and cannot appear as a producer for a required claim.

## Rust candidate verifier

A separate dependency-free Rust candidate is at `mycelix-workspace/crates/mycelix-djcc-verifier`.

Run from the repository root:

```sh
cargo +1.96.0 fmt --manifest-path mycelix-workspace/crates/mycelix-djcc-verifier/Cargo.toml -- --check
cargo +1.96.0 test --manifest-path mycelix-workspace/crates/mycelix-djcc-verifier/Cargo.toml --all-targets
cargo +1.96.0 clippy --manifest-path mycelix-workspace/crates/mycelix-djcc-verifier/Cargo.toml --all-targets -- -D warnings
```

The Rust candidate currently checks:
- all eight stage IDs are present;
- selected composition profile reference and digest match;
- one exact root scope digest/schema binding is present per stage;
- stage producer identity and source-commit syntax;
- stage output digest/schema shape;
- the fixed profile dependency graph and its canonical order;
- every dependency consumes exactly one named input binding with matching digest/schema;
- producer output digest/schema equals consumer dependency digest/schema;
- no orphan stage input bindings remain on unevaluated/not-applicable stages;
- profile-authorized `NotApplicable` only;
- no positive root claim dimension is established if any stage's corresponding claim ceiling is not established;
- deterministic reason/input ordering.

The fixed dependency graph is explicit in the source. A future profile registry may version alternative graphs, but a candidate cannot invent its own edges.

## What is not checked

The Go program validates the public seed manifest against its own expected-result table. The Rust program verifies structural composition of already supplied stage results. Neither currently verifies certificate signatures, source-log identity, authenticated owner-universe closure, Merkle root authority, semantic payload truth, legal authority, policy execution, one-shot effect admission, IVC proofs, or physical outcome.

## Shared Go/Rust stage-edge vectors

`DJCC_STAGE_EDGE_V1_KAT.tsv` specifies the first shared byte-level contract:
- domain separator `MYCELIX-DJCC-STAGE-EDGE-V1`;
- four UTF-8 strings, each prefixed by a big-endian u32 **byte** length: producer stage, consumer stage, input name, output schema reference;
- the producer's 32-byte output digest appended as raw bytes.

The Go oracle and Rust candidate independently construct the canonical preimage and compare it with the same frozen known-answer bytes. Each vector now includes the exact upstream `required_claims` set; canonical preimage encodes a u32 claim count and the sorted claim strings before the output digest. One vector contains a non-ASCII UTF-8 input name to catch character-count versus byte-count errors.

This is only a canonical stage-edge encoding vector. It is not yet a full certificate canonicalization, canonical JSON profile, or signed certificate digest. Root certificate encoding remains a separate deliverable.

The Go manifest oracle and the Rust candidate still do not compare full candidate certificate outputs against one another. Shared stage-edge byte parity is the first narrow interoperability gate.

## Claim boundary

This is **seed-registry validation plus a structural candidate verifier and one shared stage-edge byte vector**, not an end-to-end trust verifier or production assurance result. The eight seed references are symbolic until exact synthetic fixture bytes are frozen and hashed. Go/Rust edge parity does not establish complete certificate parity, external source truth, legal effect or physical outcome.


### Independence profile

The independent-corroboration profile makes source independence a first-class required claim, separate from receipt authentication and owner-universe completeness. Receipts sharing an operator, key custodian, sensor lineage, upstream feed, or control plane do not establish independent corroboration. Unknown common-cause lineage remains Indeterminate. This is still a structural candidate: no real provenance graph, field-source independence, or physical truth has been verified by this Go/Rust slice.

### MMR boundary

The planned MMR adversarial classes DJCC-009..017 are specified in the certificate document; they are not part of the eight executable symbolic seed cases and must not be reported as passed. RFC 9942 receipts prove VDS properties under a selected profile, while the MMR profile -00 remains an active Internet-Draft as of 28 November 2026.


The DJCC seed-oracle workflow prints the exact gofmt diff when its formatting gate fails so formatting fixes are evidence-driven rather than manually approximated.


## Benchmark-result qualification validator

The benchmark-run schema is at `DJCC_BENCHMARK_RESULT_V0_1.schema.json`; its dependency-free Go semantic validator is `oracle/benchmark_result.go`.

For a qualification record, the current contract requires:
- at least 30 distinct named scenario/adversarial classes;
- the exact same class set declared for positive controls and hard negatives;
- exactly one fixture record of each control kind per class;
- at least 60 executable fixture records and zero symbolic fixtures;
- unique fixture IDs and unique SHA-256 content digests;
- fixture ID, class ID, control kind, byte length, media type, generator version and deterministic seed per record;
- fixture-set digest and independent oracle version/digest;
- exact equality between fixture-record counts and the positive/hard-negative correctness denominators;
- at least 30 timed trials per stable qualification cell and all existing non-compensatory correctness gates.

The Go validator checks cross-field and set invariants that JSON Schema alone cannot express. The schema defines the structural conditional requirements; the Go validator enforces exact class pairing and fixture record reconciliation.

The validator's positive unit-test input is created by `syntheticBenchmarkResultV01`. Its hashes, byte sizes, machine details and performance values are placeholders used to test validator behavior. They are **not** actual fixture bytes, benchmark observations or a DJCC qualification result.

## Current corpus boundary

The eight entries in `DJCC_V0_1_SEEDS.json` remain symbolic seed specifications, not executable fixture packages. The 30-class/60-fixture requirement is a release contract, not evidence that those fixture files already exist. Until #6875 checks in frozen fixture bytes and both independent Go and Rust consumers read the same bytes, no corpus qualification or mutation-kill credit is claimed.

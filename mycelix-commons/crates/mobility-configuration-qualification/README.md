# Mobility Configuration Qualification Harness

This is an isolated, non-published Rust harness for **MOBILITY-COMMONS-004**.

It machine-checks the semantic qualification corpus for Mobility Configuration Contract V1. The harness intentionally stays below physical engineering and regulatory authority.

## What a green result means

A green result means:

- the corpus is structurally complete;
- all 20 qualification vectors are present exactly once;
- every vector has an explicit expected semantic outcome;
- every vector names a forbidden inference boundary;
- the qualification status remains semantic-only.

A green result does **not** establish:

- physical correctness or structural integrity;
- safety;
- road legality or seaworthiness;
- airworthiness;
- certification or manufacturing conformity;
- operational authorization.

The harness is deliberately small. It validates the qualification corpus itself rather than attempting to become a universal engineering ontology or safety oracle.

## Differential qualification

Rust and Python independently reconstruct the same semantic contract and emit a versioned normalized representation.

The differential comparator uses **structural canonicalization**, not raw file-byte equality:

1. each evaluator emits the same four semantic fields per vector;
2. vector records are canonicalized by `id`;
3. the comparator requires exactly the expected top-level and vector field sets;
4. it requires the canonical `MC-CONFIG-001` through `MC-CONFIG-020` ordering;
5. it requires the schema, version, status, cardinality, and complete vector identity set;
6. it then compares the parsed canonical JSON structures for exact equality.

This means insignificant JSON whitespace/key-order differences cannot hide a semantic mismatch, while extra fields, missing fields, duplicate IDs, omitted vectors, noncanonical normalized ordering, or changed semantic tuples fail closed.

The workflow transfers the two normalized outputs as separate GitHub Actions artifacts. GitHub's v4 artifact system makes uploaded artifacts immutable, and download performs artifact SHA-256 integrity validation. The producer jobs also publish the exact normalized-file SHA-256 as job outputs, and the differential job verifies those file hashes explicitly before parsing either result.

## Mutation matrix

The workflow also generates 20 deterministic negative corpus mutations and runs **both** evaluators against every mutation. Coverage includes missing/unknown top-level fields, wrong schema/status, missing/duplicate vectors, unknown and incorrectly typed IDs, incorrectly typed semantic fields, unknown vector fields, altered expected outcomes, altered forbidden-inference boundaries, unknown/empty scenarios, empty semantic fields, and duplicate IDs after permutation.

Input vector ordering is intentionally a separate positive invariant: the workflow reverses the checked-in corpus and requires both evaluators to normalize it back to the same canonical representation. Ordering in the source corpus is therefore not treated as semantic identity; canonical ordering is enforced at the normalized-output boundary.

## Deliberate corruption probes

The workflow mutates:

- a Rust input corpus expected outcome;
- a Python input corpus forbidden-inference boundary;
- a normalized differential result.

Each mutation must be rejected. These are negative controls: a green workflow means the harness demonstrated rejection of known-invalid changes, not that the underlying physical system is safe.

## Qualification boundary

A passing evaluator means only that the machine-readable corpus satisfies the declared semantic/structural rules. It is not an engineering analysis or certification decision.

The Holochain layer, where later integrated, may validate protocol-level structure and authorship. It must not be treated as an authority that establishes physical truth or regulatory approval.

## Cargo isolation

This harness is a standalone Cargo workspace even though its directory is nested under
`mycelix-commons`. The local `[workspace]` boundary is intentional: it prevents
Cargo from inheriting the enclosing workspace when this manifest is addressed directly.

From the repository root, use an explicit manifest path:

```sh
cargo fmt --check --manifest-path mycelix-commons/crates/mobility-configuration-qualification/Cargo.toml
cargo test --manifest-path mycelix-commons/crates/mobility-configuration-qualification/Cargo.toml
```

Or run the commands from the harness directory without a manifest path:

```sh
cd mycelix-commons/crates/mobility-configuration-qualification
cargo fmt --check
cargo test
```

Do not combine the harness working directory with a repository-root-relative
`--manifest-path`; that resolves the path relative to the harness directory.

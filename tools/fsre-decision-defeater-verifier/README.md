# FSRE independent decision-defeater verifier v0.1

**Status:** Separate E0 verifier for the first bounded `ROBUST_SAFE_ACTION_RETAINED` property. It does not depend on the decision-defeater producer crate.

## What it verifies

- A fixed-schema, length-bounded PSV manifest with a strict field order and sorted evidence references.
- SHA-256 digests over the exact raw input, report and synthetic evidence bytes, using explicit artifact-domain separation and an unambiguous 64-bit length prefix.
- Safe regular-file names, no path traversal, no symlink artifacts and explicit per-artifact size limits.
- The input's complete finite world × action grid and additive decision/safety delta mutation universe.
- All declared evidence references resolve to content-bound fixture objects.
- An independently recomputed baseline and mutated decision signature.
- Every reported minimum-cost witness, mutation/evidence mapping, cost and before/after signature.
- The global minimum and all minimum ties by a separate exhaustive enumeration of every subset (up to 22 admissible mutations).

## Run

```sh
cargo fmt --manifest-path tools/fsre-decision-defeater-verifier/Cargo.toml -- --check
cargo test --locked --manifest-path tools/fsre-decision-defeater-verifier/Cargo.toml
cargo run --quiet --locked --manifest-path tools/fsre-decision-defeater-verifier/Cargo.toml -- verify tools/fsre-decision-defeater-verifier/fixtures/pump-station
```

## Manifest schema

The manifest is UTF-8, LF-terminated, ASCII tokens only, strict field order:

```text
FSRE-DECISION-EVIDENCE-MANIFEST|1
input|pump-station.psv|sha256:<digest>
report|report.txt|sha256:<digest>
evidence|<reference>|SYNTHETIC_FIXTURE_ASSERTION|<filename>|sha256:<digest>
```

Evidence rows are sorted by reference. Filenames are single safe path components. The digest preimage is:

```text
"FSRE-CONTENT-DIGEST\0"
|| u32_be(domain_byte_length)
|| domain_bytes
|| u64_be(content_byte_length)
|| exact_content_bytes
```

Domains are `MANIFEST`, `INPUT`, `REPORT` and `EVIDENCE`. No text normalization occurs: changed whitespace or line endings in an input change its byte digest. The manifest is a versioned, fixed-schema transcript, not general JSON and not claimed to be RFC 8785 JCS. The Rust verifier uses the standard `sha2` crate; its version and all transitive dependencies are pinned in the crate-local lockfile. CI resolves these locked crates from the registry rather than claiming a pre-vendored/offline build.

## Independent semantics

This verifier independently reimplements the finite grid, safety qualification, optimal-set and minimax-regret calculations used for the report receipt. It does not call the producer's `analyze`, `apply_mask`, `compute_signature` or `replay_witness` functions.

The v0.1 verifier deliberately supports one producer property only: `ROBUST_SAFE_ACTION_RETAINED`. Other properties and mutation types return `SCHEMA_UNSUPPORTED`; they are not guessed or silently ignored.

A replayable witness is not enough to establish minimality. The verifier recomputes every subset in the declared finite universe and checks the producer's minimum cost and tie count. An exact minimum is still conditional on the supplied world and perturbation universe.

## What a green result does not mean

- The fixture evidence objects are synthetic test artifacts, not field evidence.
- SHA-256 establishes content integrity relative to the manifest, not source truth.
- The package is unsigned; issuer identity/authenticity is not established.
- Source rights and institutional authority are caller assertions, not authenticated facts.
- A log receipt or signature, if added later, would establish only the claims supported by its trust policy.
- Population completeness, causal truth, physical realizability and field safety remain not established.
- No infrastructure decision or actuation authority is granted.

The verification receipt makes these boundaries explicit.

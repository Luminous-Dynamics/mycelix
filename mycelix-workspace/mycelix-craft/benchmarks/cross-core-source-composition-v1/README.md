# Craft cross-core source composition V1

This increment addresses the source-composition prerequisite in [issue #6790](https://github.com/Luminous-Dynamics/mycelix/issues/6790).

It materializes six independently developed, standard-library-only Rust owner cores from exact PR head commits into one temporary Cargo workspace:

| PR | Owner core | Pinned commit |
|---|---|---|
| #6750 | need-intake-v1 | `3ae2f0cfeee3576f78ecf7ce6a4b8a007e51c34d` |
| #6733 | project-formation-v1 | `1bc1f221814dda2bd8786535866ed586c7114be6` |
| #6754 | claim-support-v1 | `30befb4bc5190af891cff6f820c5eb77d72fb8e6` |
| #6757 | decision-closure-v1 | `c19d12c671a4cb8ddc050dacd3ac7749e38f3314` |
| #6762 | evidence-lineage-v1 | `2d892c9eaf945ccd31d1816e4edadbbe1236a359` |
| #6769 | impact-qualification-v1 | `02c140cbaa43ae7ec4276c7edbc904a1526a4803` |

## Run

From a checkout of this repository:

```sh
rustup toolchain install 1.96.0 --profile minimal
rustup component add rustfmt clippy --toolchain 1.96.0
OUT_DIR="$(mktemp -d)"
bash mycelix-workspace/mycelix-craft/benchmarks/cross-core-source-composition-v1/materialize.sh "$OUT_DIR"
cargo +1.96.0 generate-lockfile --manifest-path "$OUT_DIR/Cargo.toml"
cargo +1.96.0 fmt --manifest-path "$OUT_DIR/Cargo.toml" --all -- --check
cargo +1.96.0 test --manifest-path "$OUT_DIR/Cargo.toml" --workspace --locked
cargo +1.96.0 clippy --manifest-path "$OUT_DIR/Cargo.toml" --workspace --all-targets --locked -- -D warnings
```

The materializer checks both the remote branch tip and the Git blob IDs for each package manifest and library source before copying them. Any branch movement or file substitution fails closed. The generated source receipt records the exact revisions and content IDs. The separate `harness` workspace member is taken from the exact harness commit checked out by CI; its manifest and source SHA-256 values are also recorded.

## Claim boundary

A green result establishes that the six pinned packages can coexist in one Cargo workspace and that each package's own unit tests, formatting and Clippy gates pass in that combined source snapshot.

## Executable cross-core slice (current)

The harness calls the public APIs of all six pinned owners and runs an independent, small service-level oracle:

- WSR-001: provider-reported completion with no current pressure-restoration or water-quality support must remain NotEligible; Decision Closure reports OpenMissingElements.
- WSR-004: current pressure and quality support may reach EligibleForLocalReview, while one of the two required mapping approvals is absent; Decision Closure reports MappingReviewRequired, never Closed.
- A source expiry must propagate NeedsRevalidation to dependent claim and decision artifacts through Evidence Lineage.
- WSR-009: an expired permit and revoked crew qualification keep the safety-critical work package BlockedBySafetyGate and the dependent work authorization NeedsRevalidation.
- Two measurements assigned to one common-cause group cannot satisfy a two-independent-source rule.

The Need Intake test constructs a valid synthetic community proposal and confirms it remains Draft without community/domain/provenance/safety review and explicit local ratification. Project Formation receives a consenting participant with no capability evidence and returns UnknownEvidence rather than a negative judgement or qualification.

The oracle is deliberately independent of the owner evaluators. The executable slice currently covers three of the 16 frozen cases (WSR-001, WSR-004, WSR-009) plus expiry propagation and common-cause independence controls. It does **not** claim the other 14 known-answer cases or all 20 registered mutations have executed.

## Claim boundary

The green result demonstrates this exact source composition plus the listed synthetic, typed cross-core flows. It is not a full WSR benchmark score, field validation, professional water-quality certification, legal authorization or permission to close a municipal service incident. The harness uses synthetic reviewer and authority references; no real authority or resident data is present.


## Benchmark protocol

See [WATER-SERVICE-CROSS-CORE-BENCHMARK-PROTOCOL-V1.md](WATER-SERVICE-CROSS-CORE-BENCHMARK-PROTOCOL-V1.md) for the baseline matrix, full-corpus qualification targets, independent-oracle requirements, water-service outcome metrics, performance reporting and field-pilot limits. It distinguishes the five-test executable slice from the remaining WSR corpus and the 20 proposed metamorphic transformations.

## Exact-head qualification result

Workflow [run #46001747298](https://github.com/Luminous-Dynamics/mycelix/actions/runs/46001747298) passed on harness head `93eadac660638db3110d8ff0190777a868768804`.

- Six source branch heads and both source blob IDs per owner were verified before materialization.
- Six owner crates passed their combined-workspace unit tests: 159 tests total.
- Five cross-core tests passed: WSR-001, WSR-004, WSR-009, expiry propagation, and common-cause independence.
- Exact-head formatting and combined-workspace Clippy passed.
- Generated Cargo.lock SHA-256: `d87aaed918e666446a61dbdb784b1c8fcb252d27d57e4fbce16320be6ae7da63`.
- Source-manifest SHA-256: `fc0a70c72967cbc366abe9aac866c097dfa96c1cf9648baad9e2a4bf685bb9fe`.

The materializer records two harness-source SHA-256 values (Cargo manifest and Rust source) and the six pinned owner manifest/source blob IDs. The temporary lockfile is regenerated by CI and its digest is recorded; it is not committed as a permanent lock for six changing upstream branches.


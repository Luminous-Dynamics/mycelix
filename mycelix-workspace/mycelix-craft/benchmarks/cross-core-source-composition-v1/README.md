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

The materializer checks both the remote branch tip and the Git blob IDs for each package manifest and library source before copying them. Any branch movement or file substitution fails closed. The generated source receipt records the exact revisions and content IDs.

## Claim boundary

A green result establishes that the six pinned packages can coexist in one Cargo workspace and that each package's own unit tests, formatting and Clippy gates pass in that combined source snapshot.

It does **not** yet establish cross-core semantic compatibility, prove that the owners exchange correct packets, or execute the 16 Water Service Repair known-answer cases and 20 mutations. Those require typed adapters, an independently written oracle and the per-owner cross-core receipts specified in #6790. No production integration or external authority is implied.

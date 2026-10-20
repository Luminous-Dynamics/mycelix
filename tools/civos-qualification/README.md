# CivOS qualification contract tool

This tool validates the CivOS multi-agent scenario contract and evaluates a per-target qualification receipt. It is **not** a hApp test adapter: it does not execute Sweettest, conductors, browser tests, or production workloads. Those adapters must produce a receipt from actual test results.

## Validate the scenario contract

Run from the repository root:

```sh
cargo run --manifest-path tools/civos-qualification/Cargo.toml -- \
  validate-contract \
  --corpus docs/research/civos-multi-agent-scenarios-v0.json \
  --schema docs/research/civos-multi-agent-scenarios-v0.schema.json
```

The command meta-validates the Draft 2020-12 schema, validates the complete corpus against it, rejects duplicate scenario IDs and hard gates without negative controls, and prints the exact-byte SHA-256 digests of the corpus and schema.

## Evaluate a receipt

A receipt must match the exact corpus/schema bytes and declare one adapter target. The receipt schema is [civos-multi-agent-receipt-v0.schema.json](../../docs/research/civos-multi-agent-receipt-v0.schema.json).

```sh
cargo run --manifest-path tools/civos-qualification/Cargo.toml -- \
  evaluate-receipt \
  --corpus docs/research/civos-multi-agent-scenarios-v0.json \
  --schema docs/research/civos-multi-agent-scenarios-v0.schema.json \
  --receipt-schema docs/research/civos-multi-agent-receipt-v0.schema.json \
  --receipt /path/to/finance-receipt.json \
  --target finance \
  --evidence-root /path/to/hashed-evidence
```

Evidence paths and artifact paths in the receipt must be relative to `--evidence-root). Absolute paths, parent-directory traversal and symlink escapes are rejected. Every referenced file is read and its bytes are SHA-256 checked against the declared digest. Each check must point to evidence included in that same scenario execution.

The evaluator then checks scenario applicability/coverage, duplicate runs, source-tree state, outcomes and required measurements. A missing, skipped, unknown, not-applicable, unrun or unsupported critical scenario is **INCOMPLETE**, never PASS. A failed hard gate vetoes qualification even if all other records claim success. Full qualification requires every scenario declared for the selected target to have a passing, evidence-backed execution and the source tree to be marked clean.

Exit codes: `0` = qualified/valid contract, `1` = failed safety or declared check, `2` = incomplete or invalid receipt/input.

## Trust boundary

- A matching hash proves byte identity relative to the supplied digest, not who produced the artifact or whether the test oracle is correct.
- The receipt is not digitally signed in v0.1; its producer and provenance still require a trusted CI/attestation layer.
- A syntactically valid receipt does not prove that the adapter exercised the real Holochain runtime. Adapters must identify runtime/HDK/HDI/client/harness versions, DNA/hApp/fixture digests and actual peer/fault settings.
- The benchmark does not imply global serializability, consensus guarantees, or cross-cluster transactions. It records results from the protocol actually implemented.
- Performance measurements are summarized separately from hard-gate safety. No SOTA or performance improvement claim is made without repeated, comparable measurements.

## Tests

```sh
cargo fmt --manifest-path tools/civos-qualification/Cargo.toml -- --check
cargo test --manifest-path tools/civos-qualification/Cargo.toml --locked
cargo clippy --manifest-path tools/civos-qualification/Cargo.toml --locked --all-targets -- -D warnings
```

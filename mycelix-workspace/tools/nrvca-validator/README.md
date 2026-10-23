# NRVCA Validator

Research-only Rust validator and adversarial mutation runner for the NRVCA v0.1 report contract.

## Run from the repository root

    cargo test --manifest-path mycelix-workspace/tools/nrvca-validator/Cargo.toml
    cargo run --manifest-path mycelix-workspace/tools/nrvca-validator/Cargo.toml -- --mode all

The default paths are repository-root-relative. Override them with --fixture, --corpus and --schema if running from another directory.

Modes:
- all: check both report and benchmark-schema contracts, validate the golden report and the design-only benchmark preregistration, then execute every adversarial mutation.
- validate: schema-contract checks plus semantic validation of the golden report and benchmark preregistration.
- corpus: execute the adversarial mutation corpus against the golden report.

Optional paths: `--benchmark PATH` and `--benchmark-schema PATH` override the default preregistration record/schema.


## Byte-verified source intake

Run `cargo run --manifest-path mycelix-workspace/tools/nrvca-validator/Cargo.toml -- hash-source --help` for the full argument contract. The `hash-source` subcommand requires a local regular file and explicit source identity, capture timestamp, extraction locator, and rights declarations. It streams the file through SHA-256 and emits an NRVCA-016-compatible source record. An optional `--expected-sha256` must match the digest actually computed from the file; mismatch exits unsuccessfully without emitting a receipt.

Example with placeholder metadata (replace these values; this does not assert any source was acquired):

```sh
cargo run --manifest-path mycelix-workspace/tools/nrvca-validator/Cargo.toml -- hash-source \
  --file ./captured.pdf --source-id source-001 \
  --publisher "Publisher name" --source-uri "https://example.org/report.pdf" \
  --publication-id "REPORT-2026" --vintage "2026" \
  --captured-at "2026-10-22T17:00:00+02:00" \
  --rights-status evaluation_only --rights-evidence-uri "https://example.org/permission" \
  --evaluation-allowed true --redistribution-allowed false \
  --locator-id "table-1-row-2" --locator "PDF page 4, Table 1, row 2" \
  --extraction-method "PDF table extraction" --tool-version "extractor-version" \
  --output ./private-source-receipt.json
```

The tool does not download the source, authenticate its publisher, adjudicate rights, verify source truth, or enable evaluation automatically. It records the canonical local path in the receipt, which may reveal machine-specific details; sanitize that field before publishing a public bundle. Hash an immutable captured copy where possible. Existing receipt paths are never overwritten.


## Content-addressed evidence capture

For source material that must remain reproducible after intake, prefer `capture-source` over `hash-source`. The latter fingerprints a file where it currently resides; the former copies and hashes the actual byte stream in one pass, then stores the result under `objects/sha256/<prefix>/<digest>` using atomic no-overwrite promotion and verification on reuse.

```sh
cargo run --manifest-path mycelix-workspace/tools/nrvca-validator/Cargo.toml -- capture-source \
  --file ./captured.pdf --store-dir ./private-evidence-store \
  --source-id source-001 --publisher "Publisher name" \
  --source-uri "https://example.org/report.pdf" --publication-id REPORT-2026 \
  --vintage 2026 --captured-at "2026-10-22T17:00:00+02:00" \
  --rights-status evaluation_only --rights-evidence-uri "https://example.org/permission" \
  --evaluation-allowed true --redistribution-allowed false \
  --locator-id table-1-row-2 --locator "PDF page 4, Table 1, row 2" \
  --extraction-method "PDF table extraction" --tool-version "extractor-version" \
  --expected-sha256 "INDEPENDENTLY-RECORDED-64-HEX-DIGEST" \
  --output ./private-evidence-store/source-001.json
```

All values above are examples, not a claim that any source was acquired or permission granted. Omit `--expected-sha256` if no independent digest exists; the captured stream is still hashed, but publisher/source authenticity remains unproven. The command does not enable evaluation.

On Unix, newly staged bytes are created owner-only, content objects are owner-read-only by default (0400), and output receipts are owner read/write (0600). The receipt uses a store-relative object path and avoids storing the input's absolute path. The store itself is not cryptographically immutable: use access controls and independently verify objects when reused. No real source was captured by the current tests.


## Read-only content-store audit

Use `verify-store` to re-hash each object in the NRVCA-018 content-addressed store, validate digest/path/shard agreement, and flag malformed entries, symlinks, digest corruption and unsafe object permissions. It never repairs or changes objects.

```sh
cargo run --manifest-path mycelix-workspace/tools/nrvca-validator/Cargo.toml -- verify-store \
  --store-dir ./private-evidence-store
```

The command prints a deterministic JSON report. `status` is `pass`, `empty`, or `failed`; `failed` returns a nonzero exit code while preserving the JSON report on stdout. `empty` is intentionally not a successful validation of evidence because it verifies no objects. See [the audit contract and benchmark protocol](../../../docs/research/nrvca/content-store-audit-v0.1.md).

## What this checks

- report status and period/jurisdiction essentials;
- source identity uniqueness and source-reference resolution;
- SNA-style account basis consistency;
- separation of financing, revaluation, secondary-market asset transfers and current production;
- unknown versus zero;
- unit consistency, canonical JSON-Pointer bindings, and independently reconstructed reconciliation identities;
- canonical stock-flow account closure and financial-event linkage;
- the v0.1 ban on adding incompatible units into a universal scalar;
- typed claim classification and the prohibition on promoting synthetic claims to observed or causal evidence;
- benchmark preregistration consistency, source-manifest/source-ID matching, source digest and licence gates, baseline/candidate arms, unique metrics, primary metric presence and retention of failed/unfavourable cases;
- evidence-tier / synthetic-to-real overclaim controls;
- every mutation is rejected in the expected structural or semantic diagnostic class.

## Limits

This is not a full JSON Schema Draft 2020-12 validator and does not prove SNA/SEEA conformity, source truth, data completeness, causal impact, or policy quality. It checks a documented subset of the contract. It has no Holochain, network, database, or runtime-service dependency. The golden fixture is synthetic.

Adversarial tests are qualified only when the Rust runner executes them successfully. A specification entry, a parsed JSON file, or an equivalent check in another language is not an executed Rust test. The claim checker uses typed claim fields; it does not attempt to infer truth from natural-language disclaimers.


The reconciliation contract binds every operand via `value_pointer` and each LHS via `lhs_pointer`; stock-flow accounts carry signed changes and reconcile independently. The validator is still a limited semantic checker, not a full JSON Schema or national-account standard validator.

A `BENCHMARK PREREGISTRATION PASS` only means the design record is internally consistent; the current South African run remains `blocked_missing_sources` until exact source bytes are pinned and rights are reviewed. It is not a benchmark result.

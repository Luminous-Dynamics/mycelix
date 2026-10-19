# Manufacturing Process-Run Receipt v0.1

## Status and purpose

This is a pure, strict evidence-envelope contract for the cross-repository process-run workflow in [MFG-QDRIFT-001](https://github.com/Luminous-Dynamics/mycelix/issues/5218). It binds one result report to exact tool/source/configuration/input identities and declares the evidence origin, model domain, assumptions, optional measurement/calibration references, independent review, and receipt ancestry.

It is an evidence reference and structural verifier. It does not create a DHT entry, authenticate an agent, hash referenced bytes, validate a model, establish a causal manufacturing improvement, or authorize machine actuation / manufacturing release.

## Paradigm shift

A run is not a free-floating metric. It is a claim attached to a precise method, build, configuration, workload, source data, and evaluation report. A timestamp or digest proves neither authorship nor truth. A simulated output is not a physical measurement. Calibration and independent review are separate, explicitly bounded claims.

The intended chain is:

```text
input/config bytes + tool/source/build identity
                 |
                 v
     typed process-run receipt (this crate)
                 |
       +---------+----------+
       |                    |
       v                    v
simulation evidence     instrument-observed evidence
       |                    |
never promoted to       observation IDs + raw-byte
measurement             digests + instrument/system ID
                            |
                            v
                    optional calibration claim
                            |
                            v
                 independent review of exact report
                            |
                            v
                 separately authorized downstream use
```

This crate implements only the receipt shape and deterministic structural-state validator. Raw-byte hashing, signer/agent binding, calibration-artifact validation, evidence publication, and downstream authorization remain separate owners.

## Contract

The new module is exported as `mycelix_bridge_entry_types::manufacturing_run_receipt`.

A version-1 `ProcessRunReceipt` includes:

- stable receipt/run/policy/scenario IDs, producer ID, deterministic seed, and creation time (Unix epoch milliseconds);
- workload SHA-256, source commit identifier (40- or 64-character lower-case Git hex), tool/version, build SHA-256, configuration SHA-256, input artifact hashes, and report SHA-256;
- explicit `evidence_origin`: `synthetic_fixture`, `simulation_output`, or `instrument_observed`;
- model domain, declared assumption IDs, and a preserved `out_of_domain` flag;
- optional measurement system, instrument, observation IDs and raw-artifact hashes, observed time, and a separate calibration claim with calibration-record digest and validity interval;
- review disposition (`not_reviewed`, `pass`, `fail`, `indeterminate`), a reviewer identity/version, exact reviewed-report digest, review time, and optional parent/superseded receipt hashes.

All SHA-256 fields are required to be 64-character lower-case hexadecimal strings. The validator checks syntax; it does not recompute any digest or establish that the referenced bytes are available. Use `decode_and_validate_process_run_receipt_json` for raw JSON so duplicate struct fields and unknown fields are rejected before the record is consumed.

## Fail-closed invariants

1. The raw-JSON decode-and-validate entry point rejects malformed JSON, duplicate struct fields, unknown JSON fields and unsupported schema versions. Blank/oversized required identities, malformed digests, invalid Git commit identifiers, duplicate artifact/observation/assumption entries, and excessive list sizes are also rejected. Use the raw decoder before consuming JSON; converting to a generic map first may discard duplicate keys.
2. Synthetic and simulated receipts cannot carry instrument observations or calibration claims.
3. Instrument-observed receipts require measurement-system and instrument IDs, observed time, observation IDs, and exactly one raw-artifact digest per observation ID.
4. A claim that calibration was current at observation requires an instrument-observed origin, calibration-record digest, an ordered validity interval, and an observation timestamp inside that interval. This validates temporal consistency only; it does not authenticate the calibration record.
5. A completed independent review requires a reviewer different from the receipt producer, a bounded reviewer version, a review timestamp, and a digest equal to the exact report digest. A review pass is not safety approval or manufacturing release.
6. An out-of-domain result remains explicitly out of domain even if the receipt has an independent review record.
7. Parent and superseding hashes provide references only. Chain completeness, invalidation on upstream changes, replay protection, source-chain author binding, and permission checks must be enforced by the responsible ledger/host integration.
8. No field or validator result grants device/machine-control authority.

## Composition boundaries

- Symtropy [QDRIFT-SYNTH-001 PR #2066](https://github.com/Luminous-Dynamics/symtropy/pull/2066) can be referenced as `simulation_output`; its synthetic process results must not be represented as observations.
- Symtropy [MFG-DATA-001 PR #2080](https://github.com/Luminous-Dynamics/symtropy/pull/2080) contains a pinned feature-level tool-wear evaluation. It is not a production KPI report and should not be forced into the process-outcome evaluator by inventing mass, energy, or duration.
- Symthaea [paired production-outcome evaluator PR #8129](https://github.com/Luminous-Dynamics/symthaea/pull/8129) can produce a referenced report artifact, but digest syntax or a structurally valid receipt does not authenticate that report.
- The existing Mycelix [ManufacturingEvidenceEntry PR #5197](https://github.com/Luminous-Dynamics/mycelix/pull/5197) is the intended artifact-reference integration target after its exact code/API is available on the chosen base. This receipt must not become a second artifact registry, evidence graph, or DHT publication subsystem.

## Proposed benchmarks

- 100% rejection of the frozen receipt mutation corpus.
- 100% of synthetic/simulation attempts to assert instrument/calibration evidence are rejected.
- 100% of invalid/stale/out-of-interval calibration claims are rejected structurally.
- 100% of review reports that reference a different report digest or reuse the producer identity are rejected.
- Deterministic serialization and validation for identical structured input under the same toolchain.
- Independent reimplementation reproduces the same structural disposition from canonical receipt bytes.
- Downstream round-trip to the existing manufacturing evidence entry preserves report digest, model/config/source identity, evidence origin, and the explicit out-of-domain state.

These are acceptance targets, not results. Current unit tests are hand-authored structural controls; cross-language independent verification and actual DHT integration are not implemented here.

## Local verification

From the Mycelix repository root:

```bash
cargo fmt --check --manifest-path crates/mycelix-bridge-entry-types/Cargo.toml
cargo test --manifest-path crates/mycelix-bridge-entry-types/Cargo.toml manufacturing_run_receipt
cargo clippy --manifest-path crates/mycelix-bridge-entry-types/Cargo.toml --all-targets -- -D warnings
```

The focused test target and full crate/workspace checks must pass at the exact PR head before this contract should be treated as implementation-qualified. A passing structural test does not establish source authenticity, measurement validity, physical process accuracy, or production benefit.

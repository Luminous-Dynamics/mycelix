# NRVCA-017: Byte-verifying source intake and acquisition receipts v0.1

**Status:** implementation draft; Rust qualification pending; no real source acquired in this workstream  
**Tracking issue:** [#5445](https://github.com/Luminous-Dynamics/mycelix/issues/5445)  
**Preceding admission contract:** [NRVCA-016](./external-validity-protocol-v0.1.md)  
**Implementation:** `mycelix-workspace/tools/nrvca-validator/src/source_intake.rs`

## Paradigm shift: hash bytes, not claims about bytes

NRVCA-016 intentionally checked only whether a manifest declared a SHA-256-shaped digest. NRVCA-017 adds a Rust `hash-source` command that opens the local file, streams it through SHA-256 in 64 KiB chunks, checks the byte count and file metadata before/after reading, optionally compares an independently obtained expected digest, and emits a JSON source record that the NRVCA-016 source-admission validator can consume.

The acquisition record retains the canonical resolved path, filename, byte count, hash method, and hash timestamp. The source record keeps the expected NRVCA-016 metadata: identity, publisher/source URI, publication ID, vintage, capture timestamp, digest, extraction method/tool version, exact locator, and explicit rights declaration. The expected digest check is case-insensitive; output digest is lowercase hexadecimal. Existing receipt output files are not overwritten.

## Usage

Build/run from the standalone validator package:

```sh
cargo run --manifest-path mycelix-workspace/tools/nrvca-validator/Cargo.toml -- hash-source \
  --file ./captured-source.pdf \
  --source-id stats-sa-p0441-q2-2026-table1 \
  --publisher "Statistics South Africa" \
  --source-uri "https://www.statssa.gov.za/publications/P0441/P04412ndQuarter2026.pdf" \
  --publication-id P0441-Q2-2026 \
  --vintage 2026-Q2 \
  --captured-at "2026-10-22T17:00:00+02:00" \
  --rights-status evaluation_only \
  --rights-evidence-uri "https://example.org/documented-permission" \
  --evaluation-allowed true \
  --redistribution-allowed false \
  --locator-id table1-cell-r1-c1 \
  --locator "PDF page 3, Table 1, row 1, column 1" \
  --extraction-method "PDF text extraction plus manual table reconciliation" \
  --tool-version "operator-recorded-extraction-tool-version" \
  --expected-sha256 "PASTE-INDEPENDENTLY-RECORDED-64-HEX-DIGEST" \
  --output ./private-source-receipt.json
```

The example metadata is illustrative; it does **not** assert that the PDF bytes were acquired, its rights were checked, or any hash is known. Omit `--expected-sha256` when no independent digest exists; the tool still computes and records the acquired file's digest, but that is not independent source authentication. A mismatched expected digest produces an error rather than a receipt.

All metadata flags are required; `--rights-status` accepts `open_reuse`, `permission_granted`, `evaluation_only`, `restricted`, `prohibited`, or `unknown`. The tool requires the rights flags to agree with the declared status. A hash does not grant evaluation or redistribution rights. Restricted/unknown sources can be fingerprinted for local inventory with both permission flags false, but NRVCA-016 will reject them for external evaluation.

## Integrity and safety boundaries

- **This computes the digest from bytes actually read**, unlike entering a digest string in JSON.
- An optional expected digest detects disagreement with an independently obtained fingerprint if one is available.
- The implementation streams rather than loading an entire PDF or dataset into memory.
- It rejects a non-regular file, malformed expected digest, invalid RFC 3339 capture timestamp in the accepted normal-seconds form (the rare `:60` leap-second notation is rejected), incoherent rights flags, and a detectable size/modification-time change during hashing.
- Output is created with create-new semantics to avoid silently overwriting an existing receipt.
- This does not prove publisher authenticity, source origin, license validity, source truth, extraction correctness, or semantic equivalence. File metadata checks reduce common concurrent-modification hazards but do not provide an operating-system snapshot guarantee; hash an immutable captured copy where possible.
- The canonical local path may disclose a username, mounted volume, or other machine details. **Treat the raw receipt as local/private evidence. Sanitize or remove `acquisition_receipt.resolved_path` before publishing the manifest**, while retaining the unmodified receipt in an appropriately controlled store. Do not commit restricted source content.
- No external evaluation is turned on by the intake command. The resulting source record still must pass independent rights review, pair adjudication, and NRVCA-016 bundle gates.

## Tests added (not yet executed here)

Unit tests cover SHA-256 known-answer vectors, actual temporary-file hashing and receipt fields, NRVCA-016 source-record admission, case-insensitive expected-digest confirmation, mismatch failure, malformed digest, rights incoherence, missing/duplicate/unknown flags, capture timestamp validation, and repeat-hash determinism.

The intended qualification gate is a Rust test pass on the exact PR head followed by real sources that are permitted for the intended evaluation. Synthetic bytes in unit tests are only software test data.

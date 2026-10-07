# FPM WASM artifact identity crosswalk

This crate defines one narrow identity boundary:

`exact artifact bytes` → `SHA-256 artifact subject` + `Holochain 0.7 WasmHash`.

The two outputs are different identity domains and are never treated as interchangeable.

## Canonical derivation

For the Holochain side, the crate delegates to `holo_hash = 0.7.0` and its typed `WasmHash` constructor. Holochain's `DnaWasm` model defines WASM hashing over the exact code bytes with `hash_type::Wasm`.

For the supply-chain side, the crate independently computes SHA-256 over those same bytes.

The resulting pair is therefore a crosswalk for one exact byte string, not a claim that the two hash values use the same algorithm.

## Match boundary

`verify_approved_artifact_against_observed_wasm_hash` requires:

1. the approved SHA-256 identity to be structurally valid;
2. the supplied artifact bytes to hash to that approved SHA-256;
3. the observed value to be a valid 39-byte Holochain `WasmHash`;
4. the observed `WasmHash` to equal the canonical Holochain derivation from those same artifact bytes.

The returned `MatchedFpmWasmArtifactIdentity` is deliberately Serialize-only and has private fields. There is no deserialization path for manufacturing a matched result.

## Nonclaims

This crate does not establish:

- build provenance;
- builder trust;
- source revision identity;
- live installation;
- live execution;
- execution race safety;
- secure boot or hardware-backed execution;
- semantic correctness.

Those remain separate boundaries tracked in FPM #4453 and #4437.

The contract also does not require the artifact bytes to be a semantically valid WASM module. Byte identity is intentionally separated from module validation and deployment qualification.
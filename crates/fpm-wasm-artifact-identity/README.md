# FPM WASM artifact identity crosswalk

This crate defines one narrow identity boundary:

`exact artifact bytes` → `SHA-256 artifact subject` + `Holochain 0.7 WasmHash`.

The two outputs are different identity domains and are never treated as interchangeable.

## Canonical derivation

For the Holochain side, the crate delegates to `holo_hash = 0.7.0` and its typed `WasmHash` constructor. Holochain's `DnaWasm` model defines WASM hashing over the exact code bytes with `hash_type::Wasm`.

The test suite contains an independent cross-check against Holochain 0.7's `DnaWasm` implementation: `holochain_raw_byte_derivation_matches_canonical_dna_wasm`. That test exists specifically to prevent a future change in the helper's byte-to-`SerializedBytes` conversion from silently changing the hash domain.

The current upstream reference audited for that canonical definition is Holochain commit `6308d28224a319546e2bb02c90e26c5c9ca10f37`. The production crate does not depend on the Holochain conductor/types crate; the canonical implementation is used only as a test oracle.

For the supply-chain side, the crate independently computes SHA-256 over those same bytes.

The resulting pair is therefore a crosswalk for one exact byte string, not a claim that the two hash values use the same algorithm.

## Match boundary

`verify_approved_artifact_against_observed_wasm_hash` requires:

1. the approved SHA-256 identity and versioned Holochain profile to be structurally valid;
2. the supplied artifact bytes to hash to that approved SHA-256;
3. the approved `WasmHash` value to equal the canonical Holochain derivation from those same artifact bytes;
4. the observed value to be a valid 39-byte Holochain `WasmHash`;
5. the observed `WasmHash` to equal the canonical Holochain derivation from those same artifact bytes.

This makes the approved pair itself a bound object rather than trusting either digest independently. A split approval record is rejected even when the runtime observation is correct.

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

## Resource bound

The verifier rejects artifacts above 16,000,000 bytes before invoking HoloHash's synchronous constructor. This matches the documented HoloHash 0.7 synchronous hashing ceiling and prevents oversized hostile input from reaching a panic-prone path.

## Dependency-closure note

The production dependency pins `holo_hash = 0.7.0`. The test-only canonical oracle additionally pins `holochain_types = 0.7.0`.

These version pins establish the intended package versions, not a reproducible dependency closure. The standalone workspace currently has no checked-in `Cargo.lock`; that is tracked separately in #4494.
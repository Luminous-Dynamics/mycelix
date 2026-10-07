# FPM WASM artifact identity crosswalk

This crate defines one narrow identity boundary:

`exact artifact bytes` → `SHA-256 artifact subject` + `Holochain 0.7 WasmHash`.

The two outputs are different identity domains and are never treated as interchangeable.

## Canonical derivation

For the Holochain side, the crate delegates to `holo_hash = 0.7.0` and its typed `WasmHash` constructor. Holochain's `DnaWasm` model defines WASM hashing over the exact code bytes with `hash_type::Wasm`.

The test suite contains a direct cross-check against Holochain 0.7's canonical `DnaWasm` wrapper: `holochain_raw_byte_derivation_matches_canonical_dna_wasm`. That test exists specifically to prevent a future change in the helper's byte-to-`DnaWasm`/`SerializedBytes` boundary from silently changing the hashable byte domain.

The exact upstream release reference is Holochain tag `holochain-0.7.0`, resolving to commit `84cdce7d4df17b95189324d5cecc3f1bfd5db30f`. This anchors the wrapper contract to the release source, not to a moving development branch. The production crate does not depend on the Holochain conductor/types crate; the canonical implementation is used only as a test oracle.

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

The approval record itself uses `serde(deny_unknown_fields)`, so an unknown/future field cannot be silently ignored during deserialization. A schema extension must therefore be explicitly versioned rather than becoming an invisible policy change.

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

The verifier rejects artifacts above `holo_hash::MAX_HASHABLE_CONTENT_LEN` before invoking HoloHash's synchronous constructor. In Holochain 0.7.0 this substrate constant is 16,000,000 bytes (16 MB), so the verifier cannot silently drift from the dependency's panic boundary.

## Dependency-closure note

The production dependency pins `holo_hash = 0.7.0`. The test-only canonical oracle additionally pins `holochain_types = 0.7.0`.

These version pins establish the intended package versions, not a reproducible dependency closure. The standalone workspace currently has no checked-in `Cargo.lock`; that is tracked separately in #4494.

## Production feature surface

The production dependency uses `holo_hash 0.7.0` with `default-features = false` and only the `hashing` feature enabled. This avoids pulling HoloHash's default Wasmer support into the verifier when no WASM runtime is executed by this crate.

The test-only `holochain_types 0.7.0` oracle is intentionally separate from the production dependency surface.

## Qualification-gate hardening

FPM qualification is isolated in `.github/workflows/fpm-wasm-artifact-identity.yml` rather than inheriting the generic CI cancellation policy. It runs for every non-draft pull request, main-branch push, or manual dispatch, so a path-filter decision cannot silently suppress identity qualification.

For pull requests, checkout is explicitly pinned to `github.event.pull_request.head.sha` and the workflow asserts that `git rev-parse HEAD` equals that exact candidate SHA. This prevents the default pull-request merge ref from being mistaken for exact-head evidence.

The qualification workflow grants only `contents: read`, disables persisted checkout credentials, and pins both checkout and the Rust toolchain actions to immutable commit SHAs. It intentionally does not configure a cross-head concurrency group, so a newer candidate cannot cancel or replace the evidence for an older exact head through workflow concurrency.

This is qualification evidence for the checked-out candidate, not proof of repository governance, protected-branch configuration, dependency reproducibility, build provenance, or live execution.

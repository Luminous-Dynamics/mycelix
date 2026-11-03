# Post-quantum provider qualification matrix — 2026-11-03

**Status:** engineering assessment and qualification plan. Not a FIPS validation, cryptographic audit, or production approval.

## Executive finding

The identity crypto crate currently exposes two different ML-DSA implementation paths:

1. The `native` feature uses `pqcrypto-dilithium` / PQClean through the legacy native wrappers. RustSec lists the pqcrypto ecosystem as unmaintained. The current native wrapper documentation identifies Dilithium3-era component sizing, while `AlgorithmId` and `TaggedSignature::new` enforce the final FIPS 204 ML-DSA-65 signature size of 3,309 bytes. Do not weaken the final-size check to make a legacy 3,293-byte signature fit.
2. The `hybrid-rc` feature uses RustCrypto `ml-dsa = 0.1.1` and `ed25519-dalek` in `hybrid_sig.rs`. It is default-off and explicitly experimental. The RustCrypto crate documents final FIPS 204 support, but also states that it has not been independently audited.

These are not interchangeable providers. A profile must pin the implementation family, version, API mode, parameter set, encoding, feature flags, build target, and evidence.

## Source inventory at the reviewed branch

| Surface | Current implementation | Qualification state |
|---|---|---|
| ML-DSA-65 / ML-DSA-87 native | `pqcrypto-dilithium` 0.5 behind `native` | Legacy provider; unmaintained ecosystem; not eligible for a FIPS 204 claim without demonstrated final-standard vector compatibility |
| SLH-DSA-SHA2-128s native | `pqcrypto-sphincsplus` 0.7 behind `native` | Legacy provider; must be checked separately against FIPS 205 vectors and exact parameter identifiers |
| ML-DSA-65 hybrid | RustCrypto `ml-dsa = 0.1.1` behind `hybrid-rc` | Experimental provider candidate; final FIPS 204 support documented; independent audit not documented |
| On-chain Holochain WASM verifier | Ed25519 host verification only | PQC component remains unverified in this path; hybrid assurance must remain partial/unsupported |
| Credential V1 signature profile | Mycelix-internal typed JSON profile | Not W3C Data Integrity, not RDF Dataset Canonicalization, and not an OpenID conformance result |

## Final-standard encoding invariants

The versioned FIPS 204 parameter-set sizes used by the current metadata layer include:

| Parameter set | Public key | Secret key | Signature |
|---|---:|---:|---:|
| ML-DSA-65 | 1,952 bytes | 4,032 bytes | 3,309 bytes |
| ML-DSA-87 | 2,592 bytes | 4,896 bytes | 4,627 bytes |

The SLH-DSA SHA2-128s profile uses a 32-byte public key, 64-byte secret key, and 7,856-byte signature. Matching these sizes is necessary but not sufficient for conformance: parameter identifiers, message/context interface, domain separation, key derivation and test-vector behavior must also match.

The current `hybrid_sig.rs` test suite now contains source-level guards for the RustCrypto ML-DSA-65 public-key and signature encoding lengths, plus negative tests for truncated encoded public keys and signatures. These tests are not executed evidence until the `hybrid-rc` feature is built and run on an exact head.

## Required ACVP campaign

Use the current NIST ACVP ML-DSA JSON specification and a pinned vector-set revision. The specification defines these ML-DSA modes:

- `keyGen / FIPS204`
- `sigGen / FIPS204`
- `sigVer / FIPS204`
- `sigGen / FIPS204-tr1`

The implementation campaign must record the exact vector-set ID, test-group IDs, test-case IDs, parameter set, mode, revision, response digest and result for every vector. At minimum:

1. **Key generation:** supplied-seed key generation must match the expected public and secret-key encoding.
2. **Signature generation:** exercise deterministic and randomized behavior as supported, internal/external interfaces, context handling, and pre-hash interfaces only where explicitly implemented.
3. **Signature verification:** accept valid vectors and reject modified signatures across the required mutation classes.
4. **Independent implementation checks:** sign with implementation A / verify with B, and sign with B / verify with A, using the same pinned profile and exact bytes.
5. **Profile-envelope checks:** reject wrong parameter sets, truncated/extended encodings, unknown algorithm/profile labels, and any attempt to reinterpret a legacy signature as final FIPS output.

NIST ACVP algorithm testing is not by itself a cryptographic module validation certificate. Do not use a local passing vector set as a FIPS 140 validation claim.

## Migration sequence

### P0 — contain legacy ambiguity

- Keep the legacy PQClean/pqcrypto native wrappers out of production-intended feature combinations.
- Preserve their historic identifiers as legacy-only, if they must remain readable.
- Never relax `TaggedSignature::new` or `AlgorithmId::signature_size` to accept a legacy size under a final FIPS identifier.
- Add explicit legacy-profile tests proving that legacy signatures cannot be emitted or accepted as final FIPS 204/205 identifiers.
- Keep all unverified WASM PQC outcomes fail-closed.

### P1 — qualify the opt-in RustCrypto ML-DSA candidate

- Pin the crate and transitive lockfile, feature set, compiler/toolchain, build target and artifact digest.
- Run the NIST ACVP campaign for ML-DSA-65; expand to ML-DSA-44 and ML-DSA-87 only after their typed envelopes and profile identifiers are complete.
- Add cross-implementation verification, malformed-encoding and message/context mutation controls.
- Measure key generation, signing and verification latency, p50/p95, peak memory, code size, signature size and WASM compatibility separately.
- Obtain independent cryptographic review before production consideration. RustCrypto's own crate warning must remain visible in the risk record until that review exists.

### P2 — qualify SLH-DSA separately

- Treat SHA2 and SHAKE families and every parameter set as separate algorithm profiles.
- Run FIPS 205 / applicable ACVP vectors and cross-implementation negative tests.
- Do not infer FIPS 205 compliance from a SPHINCS+ crate name, output length, or an older candidate implementation.

### P3 — profile and migration integrity

- Bind algorithm identifier, parameter set, provider/profile version, key identifier, public-key digest, message/prehash mode, context, and serialization profile into the signed envelope.
- Preserve legacy credentials as explicitly legacy evidence; never promote them by relabelling.
- Reissue or migrate through a separately signed migration attestation.
- Keep the internal Mycelix credential profile distinct from W3C Data Integrity suites and OpenID HAIP conformance claims.

## Benchmark protocol

Use a fixed machine and compiler profile, and run at least 30 independent process invocations per operation before publishing descriptive latency distributions. Pin payload sizes (32 B, 256 B, 4 KiB, 64 KiB), parameter set, RNG mode, CPU model, OS/kernel, compiler, target features, crate versions and warm/cold cache policy. Report throughput and p50/p95/p99, not only a single mean. Preserve raw samples and the benchmark executable digest.

Performance is a secondary gate. A faster implementation that fails standard vectors, cross-verification, malformed-input rejection, or side-channel review is not an acceptable replacement.

## References

- NIST FIPS 204: https://csrc.nist.gov/pubs/fips/204/final
- NIST FIPS 205: https://csrc.nist.gov/pubs/fips/205/final
- NIST ACVP ML-DSA JSON specification: https://pages.nist.gov/ACVP/draft-celi-acvp-ml-dsa.html
- RustCrypto `ml-dsa` 0.1.1 documentation: https://docs.rs/ml-dsa/0.1.1/ml_dsa/
- RustSec RUSTSEC-2026-0164: https://rustsec.org/advisories/RUSTSEC-2026-0164.html
- Mycelix migration issue: https://github.com/Luminous-Dynamics/mycelix/issues/5733

## Evidence ceiling

This document and the added encoding tests are an engineering assessment. No ACVP vectors were executed here; no Rust build, provider benchmark, independent cryptographic audit, Holochain WASM PQC verifier, FIPS validation, or production approval is claimed.

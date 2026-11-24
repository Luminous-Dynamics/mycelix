# Dilithium AVX2 mitigation status

**Advisory:** RUSTSEC-2026-0593  
**Affected dependency:** `pqcrypto-dilithium 0.5.0`  
**Disposition:** portable implementation only as an interim mitigation; full migration/retirement remains open.

The RustSec advisory describes two verifier discrepancies in the PQClean AVX2 implementation: a missing infinity-norm check that can accept non-conforming signatures, and a boundary-case error that rejects some valid signatures. The advisory states that disabling the `avx2` feature avoids the affected code and that no patched upstream release is expected.

The dependency declarations in these consumers therefore disable all crate default features and explicitly retain only `std`:

- `crates/mycelix-zkp-core` (`full` / `dilithium` feature path)
- `mycelix-identity/crates/mycelix-crypto` (`native` feature path)
- `mycelix-core/libs/rb-bft-consensus` (`pq` feature path)

This selects the portable/clean implementation and does not change key sizes, signature encodings, identifiers, cryptographic protocol versions, or stored data.

## What this does not establish

- It does not make the unmaintained `pqcrypto-dilithium` package maintained or remove the RustSec advisory from package metadata.
- It does not make pre-standard Dilithium2/3/5 wire formats equivalent to final NIST FIPS 204 ML-DSA-44/65/87.
- It does not qualify RustCrypto `ml-dsa`, Libcrux, or any replacement implementation.
- It does not justify an advisory ignore, a production release, or an algorithm-ID alias.
- It does not establish constant-time behavior or independent cryptographic audit.

The feature-guard workflow inspects the resolved Cargo feature graph for each enabled consumer and fails if `feature "avx2"` is present. The full package advisory remains visible to cargo-deny until the dependency is safely retired/replaced and the replacement has passed the project's protocol and independent qualification gates.

## Required migration gates

1. Resolve the algorithm identity and wire-format collision tracked by Mycelix issues #252 and #261.
2. Select an implementation of final FIPS 204 ML-DSA with explicit compatibility and maintenance evidence.
3. Run pinned FIPS/ACVP known-answer vectors, malformed-encoding tests, adversarial verifier mutation tests, and cross-implementation interop.
4. Preserve separate versioned IDs for legacy Dilithium and final ML-DSA; never silently reinterpret existing bytes.
5. Obtain independent cryptographic review/audit and publish a receipt binding source revision, dependency revision, vector digests, platforms, and outcomes.

No private keys or production health data are used by this feature-guard check.

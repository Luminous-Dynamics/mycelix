# PSI-002B3A3K0V-F2S1 validator capsule

Status: **IMPLEMENTATION GATE / NOT EXECUTED / NOT A PASS**

This directory is an intentionally standalone Cargo package for the structural JSON Schema qualification work tracked by #3439, #3440, #3443, and #3450. A compile-only crate target is present so the dependency capsule can exercise an actual locked/offline build without introducing validator logic.

## Isolation boundary

This package is outside the repository's existing Rust workspace tree. It must be invoked with an explicit manifest path:

```text
cargo +1.96.0 metadata --manifest-path qualification/psi-002b3a3k0v-f2s1-validator/Cargo.toml --locked --offline
cargo +1.96.0 build --manifest-path qualification/psi-002b3a3k0v-f2s1-validator/Cargo.toml --locked --offline
```

Repository-wide Cargo commands are **not** qualification commands.

The local `[workspace]` table makes this package its own Cargo workspace boundary rather than inheriting workspace package/dependency settings from another manifest.

## Dependency boundary

The only direct validator dependency is:

- `jsonschema = 0.58.2`
- exact version requirement;
- `default-features = false`;
- Draft 2020-12 must be selected explicitly by validator code;
- external HTTP/filesystem reference resolution is outside the qualification boundary.

A complete generated `Cargo.lock`, source inventory, and lock digest are still required before execution. The manifest alone is not a reproducibility qualification. The lock-capsule workflow also emits a canonical `provenance.v1.json` record (plus SHA-256) binding the exact source commit/tree, workflow bytes, manifest, lockfile, toolchain identity, and runner evidence; it performs locked/offline build and test checks.

## Required next gate

1. Generate the standalone lock under Rust 1.96.0.
2. Freeze its exact bytes and SHA-256.
3. Record the complete resolved source inventory.
4. Verify repeated `--locked --offline` metadata/build/test behavior.
5. Bind the resulting evidence to the exact source commit/tree, workflow bytes, manifest/lock digests, toolchain, and runner evidence.
6. Verify the canonical provenance record and capsule manifest as part of the independent audit boundary.
7. Only then replace the compile-only target with executable structural-validation code.

## Non-delegation

This package must not depend on or invoke:

- the currentness oracle;
- currentness/admission evaluator code;
- issuer-directory freshness/history code;
- token verification;
- replay/nonce authority;
- expected-output generation.

## Claim ceiling

This package boundary establishes only the intended isolation and dependency declaration.

It does **not** establish schema correctness, currentness, issuer-key admission, token validity, replay authorization, PSI qualification, or contact-discovery security.

**NOT EXECUTED / NOT A PASS.**
## Independent capsule verification

The repository also contains a dependency-free evidence consumer at `tools/verify_capsule.py`. It verifies the produced capsule without importing the generation workflow or any validator/currentness/crypto implementation. It checks the exact capsule allowlist, regular-file boundary, manifest digests, canonical provenance, source commit/tree bindings, manifest/lock digests, toolchain evidence, dependency inventory, lock projection, and package dependency boundary.

The lock-capsule workflow runs this verifier **after packaging and before artifact upload**. This creates a producer/consumer separation: the workflow constructs the evidence, while a separate implementation independently checks the packaged evidence graph.

The verifier's own PASS message is limited to **evidence integrity only**. It does not qualify JSON Schema semantics, currentness, issuer-key admission, token validity, replay authorization, PSI, or contact-discovery security.
The verifier is accompanied by `tools/test_verify_capsule_negative.py`, which first checks a synthetic baseline capsule and then requires rejection of tampering cases: provenance mutation, lock mutation, manifest mutation, missing file, extra file, symlink insertion, duplicate manifest entry, and digest mismatch. The workflow executes these tests before uploading the capsule.
## Final evidence receipt

The capsule now has a non-circular final evidence layer. A `manifest.pre-receipt.sha256` records the pre-receipt evidence set; `evidence-receipt.v1.json` then binds the canonical metadata/tree, dependency inventory, lock projection, provenance, manifest, and lockfile plus that pre-receipt manifest digest. The final `manifest.sha256` covers the receipt and pre-receipt manifest as well. This avoids self-referential hashing while ensuring the final packaged evidence is covered by two independent integrity layers.


## Pre-receipt manifest verification

The independent capsule verifier validates `manifest.pre-receipt.sha256` as an evidence structure, not merely as an opaque digest. It requires the exact base-evidence file set, deterministic sorted ordering, unique paths, valid SHA-256 records, matching file digests, and no self-reference. Negative self-tests cover reordered, duplicated, unexpected, and omitted pre-receipt entries while refreshing the outer receipt/final manifest so those cases exercise the pre-receipt parser itself.


## Lock projection structural boundary

The independent capsule verifier treats the lock/source projection as a structured
boundary rather than a set of convenient fields. It requires Cargo.lock format
version 4 with exactly the expected top-level fields, rejects duplicate full
package identities, requires the inventory's exact top-level/package record shape,
and requires canonical inventory package ordering. Lock and inventory identities
are compared with multiplicity preserved before the canonical source projection
is checked.

The generator's independent lock/source check applies the same structural
constraints before producing the projection. This prevents duplicate records,
unexpected inventory fields, or unexpected lock envelope fields from being
normalized
into an apparently valid dependency evidence state.

**NOT EXECUTED / NOT A PASS.**

## Dependency-record multiplicity

The independent capsule verifier preserves dependency-record multiplicity when projecting `Cargo.lock` and the dependency inventory. It does not collapse records into sets before comparison, so duplicate inventory records cannot be silently normalized away. The negative self-test includes an explicit duplicate inventory record while refreshing the dependent receipt and outer manifest integrity records.

## Raw JSON duplicate-member boundary

`RAW-JSON-BOUNDARY.v1.md` freezes the raw-byte parsing contract before schema
validation. Strict UTF-8 and one complete JSON value are required; duplicate
object member names are rejected recursively after escape decoding (including
escaped-name collisions). The raw SHA-256 is computed over the original bytes.
This gate deliberately does not canonicalize whitespace, object order, or
numeric spellings, and it preserves distinctions such as absent versus
`null`, empty string, empty array, and empty object.

The dependency-free reference implementation is
`tools/check_raw_json.py`; focused vectors are in
`tools/test_raw_json_boundary.py`. A later Rust implementation must enforce
the same duplicate rejection before any parser can collapse object members.

This is a syntax/evidence-identity boundary only. It does not establish schema
validity, authenticity, freshness, currentness, issuer-key admission, token
validity, replay authority, PSI qualification, or contact-discovery security.

**NOT EXECUTED / NOT A PASS.**

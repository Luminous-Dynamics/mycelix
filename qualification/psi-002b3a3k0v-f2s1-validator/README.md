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

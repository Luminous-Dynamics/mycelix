# Experimental Argon2id browser benchmark

## Purpose and boundaries

This is a **benchmark-only** experiment for comparing a memory-hard Argon2id implementation against the current WebCrypto PBKDF2 path. It does not change the production identity envelope, default KDF, unlock path, or stored identities. It is enabled only by the `argon2id-browser-benchmark` feature.

It uses the RustCrypto `argon2` 0.6.0 crate (pure Rust; supports `no_std`) inside WASM in a headless browser. The measured operation is `Argon2::hash_password_into`, which returns 32 bytes of derived key material from a synthetic passphrase and synthetic 16-byte salt.

## Parameters

- Variant: Argon2id v1.3 (version 19)
- Memory: 19,456 KiB (19 MiB)
- Iterations: 2
- Lanes: 1
- Output: 32 bytes
- Samples: 20 after one warm-up
- Timer: browser `performance.now()`
- Salt/passphrase: synthetic benchmark fixtures only; no user identity data

The memory/time/lane profile mirrors the Argon2id profile commonly recommended by OWASP for password storage. It is an initial benchmark profile, not an approved Mycelix production setting. Re-evaluate device memory limits, WASM heap pressure, event-loop blocking and deployment constraints before choosing any profile.

## Reproduce

From the repository root with the pinned `wasm-pack`, Chrome/Chromium and matching driver:

```sh
wasm-pack test --headless --chrome crates/mycelix-leptos-core --lib --locked --features argon2id-browser-benchmark -- --nocapture
```

The dedicated workflow records browser/driver versions, runs the ordinary correctness suite, measures production PBKDF2 separately, then runs this Argon2id feature. It checks that lockfiles do not change during each benchmark invocation.

## Comparison methodology

The production PBKDF2 report now includes a `derive_bits_ms_p50/p95/max` series using WebCrypto `deriveBits`; the synthetic password is imported before the timed loop so each sample measures the KDF operation. It also reports end-to-end PBKDF2 encrypt and unlock/decrypt timings.

This Argon2id report's `derive_ms_p50/p95/max` measures Argon2id `hash_password_into` only. This comparison deliberately exposes the browser implementation distinction:

- PBKDF2 runs in the browser's native WebCrypto implementation.
- Argon2id runs as pure Rust compiled to WASM and executes synchronously in the browser test thread.
- Both use synthetic password/salt inputs and produce 32 bytes. The PBKDF2 path imports the password key once outside its timed loop; Argon2 needs no separate base-key import.
- Do not interpret the result as a universal algorithm speed ranking. The browser, CPU, driver, WASM optimization profile and JS/WASM boundary all influence the result.
- Do not call the KDF faster/safer from one GitHub runner. Compare p50/p95/max and memory use on declared desktop and low-end/mobile browser targets.

## Required evaluation before any production consideration

1. Add versioned KDF/cipher metadata and authenticated bounds for Argon2id memory/time/lanes. Unknown or excessive parameters must fail closed before allocating memory.
2. Add cross-version encrypt/decrypt vectors, wrong-passphrase and tampered-metadata negatives, and explicit migration/rollback semantics.
3. Measure browser UI stalls, WASM code-size impact, peak memory, multiple tabs and failure on constrained devices.
4. Evaluate a Web Worker execution strategy before enabling synchronous memory-hard KDF work in a production UI.
5. Keep PBKDF2 as the existing production format until a reviewed, versioned transition and target-device evidence supports changing it.

## References checked 2026-10-20

- OWASP Password Storage Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/Password_Storage_Cheat_Sheet.html
- RustCrypto Argon2 crate 0.6.0 documentation: https://docs.rs/argon2/0.6.0/argon2/
- RustCrypto Argon2 source / feature flags: https://docs.rs/argon2/0.6.0/argon2/

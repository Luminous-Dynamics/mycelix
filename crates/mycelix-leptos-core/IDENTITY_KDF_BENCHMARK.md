# Identity KDF latency benchmark

## Purpose

Measure Mycelix's **production** browser key-wrapping cost before changing KDF settings. This is a latency calibration, not a password-cracking benchmark, penetration test, or certification.

Current production envelope policy uses PBKDF2-HMAC-SHA256 at 600,000 iterations and AES-256-GCM. The versioned envelope authenticates the algorithm/work-factor header and bounds accepted work-factor values. The production benchmark feature forces 600,000 iterations even while the browser test binary is compiled with `cfg(test)`; the test asserts this before measuring.

## Reproduce

Install the repository-pinned `wasm-pack` version and ensure Chrome/Chromium plus its matching driver are installed. From the repository root:

```sh
wasm-pack test --headless --chrome crates/mycelix-leptos-core --lib --locked --features production-kdf-benchmark production_kdf_latency_benchmark
```

The test uses the browser's monotonic `performance.now()` timer, performs one warm-up encrypt/decrypt pair, and then collects 20 paired samples on a synthetic 32-byte test secret and a synthetic passphrase. It emits one machine-readable `IDENTITY_KDF_BENCHMARK` line. Do not supply actual user credentials, use a real mnemonic, or add secret-derived output.

The percentile/statistics helper is tested independently on native Rust without invoking WebCrypto:

```sh
cargo test --manifest-path crates/mycelix-leptos-core/Cargo.toml --locked --lib --features production-kdf-benchmark benchmark_statistics_tests
```


The exact-head CI workflow additionally records the Chrome/Chromium and ChromeDriver versions, runs the normal quick correctness suite independently, runs the production-factor benchmark as a separate step, and verifies that neither test command changes the generated lockfile.

## What is measured

- `encrypt_ms`: browser WebCrypto encrypt operation including salt/IV generation, PBKDF2 derivation, and AES-GCM encryption.
- `unlock_decrypt_ms`: browser WebCrypto decrypt operation including PBKDF2 derivation and authenticated AES-GCM decryption. This isolates the expensive cryptographic path; it does not include the full Personal startup, localStorage inspection, DOM rendering, or Holochain conductor connection.
- `p50`: nearest-rank 50th percentile of the 20 samples (lower-middle observation for this even sample count).
- `p95`: nearest-rank 95th percentile of the 20 samples.
- `max`: maximum observed sample. With only 20 samples, p95 and max are close; use a longer controlled run for device qualification.

## Interpretation / decision rules

1. A GitHub-hosted Chrome measurement is useful as a reproducible CI baseline, not as a proxy for every low-end Android device or desktop.
2. Record device/browser/OS and cold/warm conditions when collecting manual target-device measurements. Compare p50/p95, retry rate, and successful recovery—not only one fast benchmark.
3. Initial product hypothesis: p95 decrypt/unlock cryptographic work under 1.5 seconds on a declared reference device. This is a target to test, not a measured result or security standard. Revise only with user/device evidence and an explicit security trade-off.
4. Never reduce the production work factor just to meet a latency target. If target devices cannot meet the target, evaluate a reviewed, versioned alternative KDF and its WASM footprint instead.
5. OWASP prefers Argon2id when compatible; PBKDF2-HMAC-SHA256 at 600,000 iterations is its stated FIPS-oriented fallback recommendation. Argon2id is **not implemented or benchmarked by this harness yet**. Any future implementation must use a reviewed library, fixed/bounded parameter policy, explicit envelope version/identifier, authenticated metadata, migration tests, and target-device measurements.
6. Evaluate WebAuthn/passkeys as optional user verification or key-wrapping input in a separate prototype. A passkey credential must not be treated as the Mycelix Ed25519 signing key, and it must not replace the independent offline recovery backup without a protocol-level recovery analysis.
7. Passphrase-wrapped local storage does not defend a live unlocked page from active XSS, malicious extensions, or a compromised browser profile.

## Standards and references checked 2026-10-20

- OWASP Password Storage Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/Password_Storage_Cheat_Sheet.html
- NIST SP 800-63-4 (final July 2025): https://csrc.nist.gov/pubs/sp/800/63/4/final
- W3C WebAuthn Level 3 publication history (Recommendation 25 August 2026): https://www.w3.org/standards/history/webauthn-3/
- OWASP HTML5 Security Cheat Sheet (localStorage/XSS risk): https://cheatsheetseries.owasp.org/cheatsheets/HTML5_Security_Cheat_Sheet.html

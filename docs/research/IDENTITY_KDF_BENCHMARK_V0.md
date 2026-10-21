# Mycelix identity KDF production benchmark v0

**Status:** benchmark harness only. It produces a real-browser CI timing baseline; it does not yet establish cross-device latency, password strength, or security certification.

## Why this is separate from the identity unit tests

The shared core sets `PBKDF2_ITERATIONS = 600_000` for non-test builds and a deliberately reduced count of `1_000` for the core crate's own test configuration so routine crypto tests complete quickly. If the core is built as an ordinary dependency of a separate benchmark package, its `cfg(test)` flag is not enabled. The benchmark-only feature also checks that exactly 600,000 iterations are active before it records results.

The isolated package is not a dependency of Personal, Finance or Climate. It is only built when the benchmark workflow is invoked.

## What it measures

The headless-Chrome browser test runs 15 measured samples per metric, after unrecorded warm-up calls:

- `kdf_ms`: PBKDF2-HMAC-SHA256-derived non-extractable AES-GCM key derivation only; fresh salt generation is outside the timer.
- `decrypt_ms`: end-to-end decryption, including KDF, AES-GCM authentication and plaintext recovery for a fixed synthetic test payload.

The test logs a `IDENTITY_KDF_BENCHMARK_RECEIPT=` JSON line with the iteration count, sample count, browser user-agent, raw durations in milliseconds, min/median/P95/max and an explicit non-secret marker. It never accepts a user's recovery phrase, identity seed, actual passphrase, or real ciphertext. Do not put user-identifiable data into the receipt.

The work-factor check prevents an accidentally test-configured build from pretending to measure production PBKDF2. Native tests and ordinary core browser tests do not replace this measurement.

## Proposed target and comparison methodology

Don't reduce the 600,000-round work factor just to hit a latency target. First collect baselines from:

1. CI headless Chrome on the pinned Ubuntu runner (repeatable development baseline).
2. Supported desktop browsers and OSes, including at least Chrome/Chromium, Firefox and Safari where the actual application can be tested.
3. Representative lower-end Android/mobile devices and private/incognito modes where relevant.
4. Storage-denied, WebCrypto-unavailable and cancelled/failed unlock paths as separate correctness cases—not as fast timing samples.

For each environment, record P50/P95, sample count, run count, browser/OS versions, CPU/device class when explicitly known, background-tab state and thermal/power context when possible. Avoid mixing samples from different machines into one distribution.

**Initial performance objective to evaluate, not a current claim:** median full unlock below 500 ms and P95 below 1,000 ms on the declared supported desktop reference set. If production measurements miss the objective, test a versioned Argon2id alternative and benchmark its memory/CPU bounds on the same hardware before proposing migration. These targets are a UX hypothesis, not a NIST or OWASP mandate. FIPS-constrained deployments may require a different choice.

## Industry baseline and decision rule

OWASP prefers Argon2id for password hashing where available and recommends PBKDF2-HMAC-SHA256 with 600,000 or more iterations for FIPS-oriented use. The current Mycelix setting is therefore a defensible PBKDF2 baseline, but we still need actual latency data and platform-specific support assessment. A raw speed comparison alone does not decide the algorithm: memory hardness, compliance needs, portability, energy cost, downgrade/legacy migration, and the user's ability to recover the same DID all matter.

- OWASP Password Storage Cheat Sheet: https://cheatsheetseries.owasp.org/cheatsheets/Password_Storage_Cheat_Sheet.html
- NIST SP 800-63-4: https://csrc.nist.gov/pubs/sp/800/63/4/final

## Nonclaims

- CI Chrome is not a representative mobile benchmark.
- 15 samples on one CI run is a preliminary baseline, not a robust P95 estimate or a SOTA comparison.
- No Argon2id runtime is implemented by this PR.
- Timing data does not prove cryptographic strength, resistance to XSS, or recoverability.
- Browser test compilation without execution does not qualify the benchmark.

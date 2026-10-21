# Mycelix Personal Identity Security Qualification v0.1

**Status: specification only; the corpus below has not yet been executed as a whole.** It is a frozen contract for adversarial qualification, not a security certification.

## Paradigm shift

Qualification is a state-transition proof obligation, not a UI checklist or a count of unit tests. The security boundary is the identity state machine plus checked storage transitions; the front end is a caller that must obey a shared action policy. Tests must cover both direct API calls and app-level route gating.

The core safety invariant is: **identity loss or uncertainty never authorizes silent replacement**. `StorageUnavailable`, legacy labels, partial key material, mismatched key pairs, stale session cache, storage races and failed rollback remain distinct states. A successful storage API call is not enough: the intended postcondition must be read back, verified, and cryptographically bound to the existing public key.

## Research basis and industry comparison

- **W3C WebAuthn Level 3** became a Recommendation on 25 August 2026. It defines RP-scoped, public-key credentials and browser-mediated authenticator consent: https://www.w3.org/TR/2026/REC-webauthn-3-20260825/
- **NIST SP 800-63-4** is final (July 2025) and differentiates phishing resistance, replay resistance and syncable-authenticator trade-offs: https://csrc.nist.gov/pubs/sp/800/63/4/final and https://pages.nist.gov/800-63-4/sp800-63b/authenticators/
- The **W3C WebAuthn PRF explainer** describes an optional per-credential PRF output that can be used for local key wrapping, while requiring a WebAuthn ceremony and noting the scope/privacy constraints: https://github.com/w3c/webauthn/blob/main/explainers/prf-extension.md
- **OWASP** warns that same-origin JavaScript can read/poison localStorage and that XSS can expose stored secrets; encrypted-at-rest does not solve active page compromise: https://cheatsheetseries.owasp.org/cheatsheets/HTML5_Security_Cheat_Sheet.html
- **W3C Verifiable Credentials Data Model 2.0** provides portable credential semantics and status but does not by itself establish issuer trust or eliminate correlation: https://www.w3.org/TR/vc-data-model/

### Where Mycelix can be better

Do not claim to outperform industry by inventing another crypto primitive. Compete on **failure-state semantics, no-silent-replacement guarantees, exact-head repeatability, multi-device recovery, accessibility and transparent evidence**. Most benchmarks should first be binary safety gates; only report unlock latency, cancellation and recovery completion after those gates pass.

## Hard gates

1. **Zero false `NoIdentity` outcomes:** every storage read error and every partial/corrupt shape must be reported as unavailable or inconsistent.
2. **Zero silent overwrites:** an existing identity, legacy marker, or in-memory key blocks protected creation/recovery unless a separately reviewed explicit replacement protocol is used.
3. **Zero cache-on-failure:** failed unlock, wrong passphrase, tampered ciphertext, missing public metadata or public-key mismatch leaves no signing key in the session cache.
4. **Zero plaintext persistence in protected posture:** no raw seed or secret-key encoding in localStorage/IndexedDB, URL, logs, telemetry, public DHT records or diagnostic receipts.
5. **100% postcondition verification:** create, protect, restore and rotate report success only after read-back and key/public binding checks succeed.
6. **100% replay/origin rejection** for any future WebAuthn path, before it can be presented as supported.
7. **Zero weakened gates:** queued, skipped, flaky, stale-SHA, native-only or compile-only results do not qualify browser security behavior.

## Measurements

Record P50/P95 protected unlock latency, passphrase and passkey cancellation rates, restore completion rate, storage failure recovery rate, and compatibility by exact browser/OS/authenticator version. Keep separate metrics for (a) operation attempted, (b) cryptographic validation passed, (c) state persisted and read back, and (d) resulting state recognized after reload. Performance never compensates for a failed safety gate.

## Evidence contract

Each future execution receipt must include the exact source SHA, test runner/version, target browser and OS, authenticator class where applicable, scenario ID, expected postcondition, observed result, failure classification, and a digest over a **non-secret** receipt. It must not contain a passphrase, recovery phrase, private key, PRF output, decrypted data or raw health information.

Run native unit tests for pure state policy/classification, real-browser tests for WebCrypto/storage semantics, fault-injection tests for write/readback rollback, multi-tab races for async transitions, and cross-origin/capability tests for WebAuthn. These are distinct layers; one does not substitute for another.

## Scenario status

The machine-readable corpus currently defines 43 synthetic cases. All are initially marked `specified_not_executed`. This state is intentional; do not edit a scenario to `pass` without a source-SHA-bound receipt. Future WebAuthn cases are requirements for an experimental feature and are not implemented support claims.

## Files

- `identity-security-scenarios-v0.json` — frozen cases and evidence requirements.
- `identity-security-scenarios-v0.schema.json` — JSON Schema Draft 2020-12 for corpus validation.

## Nonclaims

This document is not a WebAuthn/VC conformance statement, independent security audit, formal proof, identity assurance level, or statement that the listed browser tests have run. Local passphrase encryption does not prevent a compromised same-origin script from using or reading an already-unlocked identity.
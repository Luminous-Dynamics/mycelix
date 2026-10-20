# Mycelix Identity and Recovery Qualification Plan

**Status:** engineering targets, not achieved-results claims  
**Scope:** browser-local identity bootstrap, passphrase protection, backup restore, DID-controller recovery, and consented credential exchange  
**Last standards review:** 2026-10-20

## Thesis: make identity a verified state machine, not a side effect

The product goal is **recoverable self-custody with no silent identity replacement**. The app must first inspect its local state, then permit only state-specific actions, and finally verify the resulting state before granting access to identity-dependent features.

A signature, a KDF, a passkey, a registry entry, and a UI success toast prove different things. Keep these claims separate:

- **Key integrity:** the decrypted Ed25519 key derives the public key stored for this identity.
- **At-rest protection:** the persisted secret is encrypted and no plaintext secret copy remains in the declared storage.
- **Session readiness:** a matching key is in memory in this session.
- **Controller authority:** the current DID registry/controller rules authorize the actor to update that DID.
- **Recovery assurance:** the recovery ceremony has authenticated enough independent evidence and is bound to the intended DID, request, new controller, and nonce.
- **Credential truth:** a claim's issuer, authority scope, status and freshness are distinct from its signature validity.

Do not collapse these into a single `verified=true` flag.

## Current baseline and limits

The current implementation uses a browser-generated Ed25519 key, a local BIP-39 backup phrase, Web Crypto PBKDF2-HMAC-SHA256, and AES-256-GCM for passphrase-based key wrapping. Production currently uses 600,000 PBKDF2 iterations; browser unit tests deliberately use 1,000 iterations under `cfg(test)`. See `crates/mycelix-leptos-core/src/identity_crypto.rs`.

That is a useful at-rest control, but **not an XSS defense**. OWASP warns that JavaScript executing in an origin can read or modify Web Storage; a compromised active page may access plaintext in memory or invoke cryptographic operations even when the derived Web Crypto key is non-extractable. The bootstrap gate must not imply protection from active XSS, malicious extensions, or an already-compromised browser profile.

The protected-create and restore APIs verify read-back and refuse to overwrite recognized local identity state. The posture inspector is diagnostic; envelope-shape checks alone do not authenticate encrypted contents without the passphrase. Browser-only tests must actually run in an isolated browser before claiming WebCrypto/storage-transition qualification.

## Recommended architecture shift

### 1. From imperative initialization to a pure policy matrix

Make the allowed action set a pure function of posture. No screen component, route, or provider is allowed to invent its own transition rules.

| Observed posture | Allowed bootstrap mutation | Forbidden defaults |
|---|---|---|
| No identity | Create protected; restore protected backup | Never restore over an existing key |
| Unprotected identity | Protect that same key | Never mint a replacement |
| Protected, locked | Unlock | Never create or restore |
| Protected, unlocked in this session | Rotate passphrase after re-verification | Never trust a stale cache if stored public-key binding changed |
| Legacy random-label identity | None; explicit migration flow later | No silent replacement |
| Storage unavailable | None | Never interpret as no identity |
| Corrupt/inconsistent | None | Never auto-repair by key generation |

The policy matrix should be exhaustively tested. Storage mutation functions still repeat preconditions and read back what they wrote; UI validation is not a security boundary.

### 2. From “encrypted localStorage” to a layered device-and-recovery model

Use distinct controls for distinct threats:

- **At-rest theft:** passphrase-wrapped key, authenticated encryption, high-entropy random salt/nonce, documented KDF parameters and key format.
- **Phishing / deliberate user-presence proof:** evaluate WebAuthn Level 3 as an optional, origin-scoped, user-mediated authorization factor. Do not treat a WebAuthn credential as the Holochain Ed25519 agent key: a WebAuthn authenticator keeps its own RP-scoped key and exposes assertions, not an arbitrary export of the wallet key.
- **Portability:** retain an explicit offline recovery path with a verified export/restore round trip and clear warnings about seed-phrase possession.
- **Recovery from loss:** design a DID-controller transfer ceremony with fresh nonce, expiry, request/DID/new-controller binding, replay protection and auditable completion. If social recovery is added, specify independent factors, threshold assumptions and collusion/failure cases before UI implementation.
- **Compromised active origin:** CSP, Trusted Types where supportable, dependency minimization, no dynamic script injection, separation of hApps across origins, and security tests. Passphrase wrapping alone cannot solve this threat.

### 3. From opaque crypto bytes to a versioned, migratable envelope

Before changing algorithms or work factors, add an explicit envelope version and authenticated metadata for the KDF/cipher identifiers and work factor. Preserve a legacy decoder for the existing three-field shape. Reject unknown versions and algorithms before trying decryption; never silently downgrade.

Potential envelope fields (illustrative contract, not yet a migration):
- `version`
- `kdf` and validated `iterations`
- `cipher`
- `salt_b64`, `iv_b64`, `ciphertext_b64`
- optional public-key fingerprint for mismatch detection (not a replacement for deriving and comparing the public key)

Benchmark the production work factor on target devices. Do not reduce it for speed below the reviewed security floor. A future Argon2id path may be worth evaluating, but Web Crypto does not expose it as a native KDF; introducing a WASM implementation adds supply-chain, memory, compatibility and audit costs. Select it only from measured evidence and a documented upgrade/legacy strategy.

## Industry and standards positioning

| Area | Public baseline / relevant reference | Mycelix goal |
|---|---|---|
| Web credential shape, issuer/holder/verifier roles, status and privacy | [W3C VC Data Model 2.0](https://www.w3.org/TR/2025/REC-vc-data-model-2.0-20250515/) is a Recommendation | Interoperate with credential protocols; keep internal provenance, freshness, authority scope and consent semantics richer than a single valid/invalid bit |
| Credential presentation and issuance | [OpenID for Verifiable Presentations 1.0](https://openid.net/openid-for-verifiable-presentations-1-0-final-specification-approved/) (Final Spec approved July 2025) and [OpenID for Verifiable Credential Issuance 1.0](https://openid.net/openid-for-verifiable-credential-issuance-1-final-specification-approved/) (Final Spec approved September 2025) | Implement protocols only where real issuer/verifier adapters exist; never label a local synthetic card as interoperable |
| Origin-scoped authenticator proof and user-mediated operations | [W3C WebAuthn Level 3](https://www.w3.org/TR/2026/REC-webauthn-3-20260825/) became a Recommendation on 25 August 2026 | Offer as a separate proof/authorization factor; do not conflate its RP-scoped key with the Holochain agent key |
| Storage and active-script threat model | [OWASP HTML5 Security Cheat Sheet](https://cheatsheetseries.owasp.org/cheatsheets/HTML5_Security_Cheat_Sheet.html) warns that XSS can read and write Web Storage | Claim the precise protection boundary; never describe localStorage encryption as XSS-safe |
| Key lifecycle, KDF and rotation | [OWASP Cryptographic Storage Cheat Sheet](https://cheatsheetseries.owasp.org/cheatsheets/Cryptographic_Storage_Cheat_Sheet.html) and [Key Management Cheat Sheet](https://cheatsheetseries.owasp.org/cheatsheets/Key_Management_Cheat_Sheet) | Explicitly test generation, rotation, restore, corruption, zeroization best-effort and failed cleanup |
| Digital identity proofing, authentication and federation | [NIST SP 800-63-4](https://csrc.nist.gov/pubs/sp/800/63/4/final), finalized July 2025 | Treat as a reference for assurance and lifecycle requirements, not as a certification claim for Mycelix |

Sources establish interoperability and threat-model requirements; they do not certify this code or imply that Mycelix complies with a standard.

## Qualification benchmarks

These are target criteria, not claims of current attainment.

### P0 — no silent identity loss

1. **Transition matrix:** 100% of posture/action pairs match the policy table. Include negative tests for every blocked action.
2. **Overwrite resistance:** 0 replacements of any non-empty recognized identity state in 10,000 deterministic randomized state/interleaving cases.
3. **No-identity false positives:** storage exceptions, malformed JSON, partial key pairs and legacy labels produce no automatic key generation in 100% of frozen cases.
4. **Fail-closed unlock:** wrong passphrase, altered ciphertext, missing public key, mismatched decrypted key and invalid envelope never populate the in-session key cache.
5. **At-rest plaintext:** after successful protect/create/restore, 0 plaintext secret-key entries remain in the audited storage keys; if cleanup/read-back is ambiguous, the result is an explicit error and posture remains gated.
6. **Recovery integrity:** every successful recovery receipt binds DID, request ID, new controller key, nonce, expiry, authorization evidence and completion event; replaying the receipt is rejected.
7. **Adversarial coverage:** two-agent negative tests for wrong DID, wrong controller, copied anchor, stale request, replayed nonce, duplicate vote, expired request and config shadowing. 100% expected decisions on the frozen corpus.

### P1 — usable protection without weakening the floor

1. Production PBKDF2 work factor is never below the reviewed minimum. Record p50/p95 unlock latency, browser, CPU class and device class.
2. Engineering target: p50 decrypt/unlock under 500 ms and p95 under 1.5 s on the declared reference-device set. If current 600,000-iteration PBKDF2 misses that, do not lower the floor silently: profile UI latency and document an explicit format-versioned KDF decision.
3. Protected create, protect-existing, restore, unlock and passphrase rotation preserve the same expected public key when they are meant to preserve identity.
4. User-visible error copy distinguishes locked, unavailable, corrupt and legacy states without exposing whether a particular passphrase was close to correct.
5. Accessibility: all bootstrap controls keyboard-operable, status changes announced, no keyboard trap, 200% zoom/reflow at 320 CSS pixels, and reduced-motion preference respected.

### P2 — privacy and interoperability

1. No raw recovery phrase, passphrase, plaintext key, medical values or unneeded stable identifiers appear in logs, telemetry, URL, clipboard, analytics or public DHT entries.
2. Disclosure preview defaults to no fields. A recipient must see categories, purpose, expiry, recipient identity/authority status and which fields will be sent before an actual presentation.
3. Run protocol fixtures for supported W3C VC/OpenID profiles with explicit issuer trust and status checking; unsupported formats fail closed.
4. Report selective-disclosure coverage and cross-presentation correlation risk by credential profile; do not claim unlinkability solely because a credential is signed.

## How to make the benchmarks scientifically credible

- Freeze test fixtures and expected outcomes, hash the corpus and store the corpus digest in the qualification output.
- Run each case at the exact commit SHA; record browser/OS/device, Rust version, lockfile digest, build flags and test-run identifier.
- Include positive controls, negative controls, corrupted storage, unavailable storage and race/interleaving cases. A queued workflow is not a pass.
- Separate native state-machine tests from real-browser WebCrypto tests and multi-agent Holochain integration tests; one test class does not substitute for another.
- Publish failures and regressions as well as medians. Never tune the metric by excluding a failing device or weakening a negative control without a reviewed rationale.
- Use external independent review for recovery logic and cryptographic key lifecycle before making production-safety claims.

## Next implementation order

1. Pure bootstrap action policy with exhaustive tests.
2. Exact-head native tests, strict Clippy and WASM compile.
3. Run protected create/protect/restore/rotate/tamper cases in isolated real browsers.
4. Version the encryption envelope with explicit legacy decoding and upgrade tests.
5. Add the UI bootstrap gate against the shared policy rather than duplicating state rules.
6. Add multi-agent DID-recovery and consent-enforcement regressions.
7. Evaluate WebAuthn as an additional origin-scoped authorization factor; only then decide whether it meaningfully improves Mycelix's threat model.
